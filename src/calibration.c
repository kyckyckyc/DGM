/*
    Copyright 2021 codenocold codenocold@qq.com
    Address : https://github.com/codenocold/dgm
    This file is part of the dgm firmware.
    The dgm firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    The dgm firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "calibration.h"
#include "anticogging.h"
#include "can.h"
#include "encoder.h"
#include "foc.h"
#include "heap.h"
#include "mc_task.h"
#include "pwm_curr.h"
#include "usr_config.h"
#include "util.h"

typedef enum eCalibStep {
    CS_NULL = 0,

    CS_MOTOR_R_START,
    CS_MOTOR_R_LOOP,
    CS_MOTOR_R_END,

    CS_MOTOR_L_START,
    CS_MOTOR_L_LOOP,
    CS_MOTOR_L_END,

    CS_DIR_PP_START,
    CS_DIR_PP_LOOP,
    CS_DIR_PP_END,

    CS_ENCODER_START,
    CS_ENCODER_CW_LOOP,
    CS_ENCODER_CCW_LOOP,
    CS_ENCODER_END,

    CS_REPORT_OFFSET_LUT,
} tCalibStep;

#define MAX_MOTOR_POLE_PAIRS 30U
#define SAMPLES_PER_PPAIR    128U

static int       *p_error_arr = NULL;
static tCalibStep mCalibStep  = CS_NULL;

static volatile float current_samples[21 * SAMPLES_PER_PPAIR];  // MAX_SAMPLES需要根据实际情况定义
static volatile float ENCODERtest_sample[21 * SAMPLES_PER_PPAIR]; 
static volatile float count_ref_sample[21 * SAMPLES_PER_PPAIR];
void CALIBRATION_start(void)
{
    // free
    if (pCoggingMap != NULL) {
        AnticoggingValid = false;
        HEAP_free(pCoggingMap);
        pCoggingMap = NULL;
    }

    // malloc
    if (p_error_arr == NULL) {
        p_error_arr = HEAP_malloc(SAMPLES_PER_PPAIR * MAX_MOTOR_POLE_PAIRS * sizeof(int));
    }

    UsrConfig.encoder_dir = +1;
    UsrConfig.calib_valid = false;
    mCalibStep            = CS_MOTOR_R_START;
}

void CALIBRATION_end(void)
{
    FOC_disarm();

    mCalibStep = CS_NULL;

    // free
    if (p_error_arr != NULL) {
        HEAP_free(p_error_arr);
        p_error_arr = NULL;
    }

    // malloc
    if (0 == USR_CONFIG_read_cogging_map()) {
        AnticoggingValid = true;
    } else {
        USR_CONFIG_set_default_cogging_map();
        AnticoggingValid = false;
    }
}

void CALIBRATION_loop(void)
{
    static uint32_t loop_count;

    // R
    static const float    kI           = 0.05f;
    static const uint32_t num_R_cycles = CURRENT_MEASURE_HZ * 2;// 电阻测量周期数(2秒)

    // L
    static float          Ialphas[2];
    static float          voltages[2];
    static const uint32_t num_L_cycles = CURRENT_MEASURE_HZ / 2;// 电感测量周期数(0.5秒)

    static const float calib_phase_vel = M_PI;                  // 校准时的相位速度(π rad/s)

    static float phase_set;                                     // 设置的相位角
    static float start_count;                                   // 起始编码器计数值

    static int16_t sample_count;                                // 采样计数器
    static float   next_sample_time;                            // 下次采样时间

    float       time    = (float) loop_count * CURRENT_MEASURE_PERIOD;                                     // 当前时间
    const float voltage = UsrConfig.calib_current * UsrConfig.motor_phase_resistance * 3.0f / 2.0f;        // 计算电压=校准电流*相电阻*3/2
    const float voltageencoder = 2*UsrConfig.calib_current * UsrConfig.motor_phase_resistance * 3.0f / 2.0f;

    switch (mCalibStep) {
    case CS_NULL:
        break;

    case CS_MOTOR_R_START:
        loop_count  = 0;
        voltages[0] = 0.0f;
        mCalibStep  = CS_MOTOR_R_LOOP;
        break;

    case CS_MOTOR_R_LOOP:
        voltages[0] += kI * CURRENT_MEASURE_PERIOD * (UsrConfig.calib_current - Foc.i_a);    // PI控制器计算电压：通过电流闭环控制来测量电阻

        // Test voltage along phase A   沿A相施加测试电压
        FOC_voltage(voltages[0], 0, 0);
        // 达到预定周期数后结束电阻测量
        if (loop_count >= num_R_cycles) {
            PWMC_TurnOnLowSides();// 开启下桥臂，停止电机
            mCalibStep = CS_MOTOR_R_END;
        }
        break;

    case CS_MOTOR_R_END:
        UsrConfig.motor_phase_resistance = (voltages[0] / UsrConfig.calib_current) * 2.0f / 3.0f;    // 计算相电阻: R = V/I，并转换到相电阻值
        {
            uint8_t data[4];
            float_to_data(UsrConfig.motor_phase_resistance, data);
            CAN_calib_report(1, data);      // 通过CAN报告电阻值
        }
        // mCalibStep            = CS_NULL;
        // UsrConfig.calib_valid = true;
        // MCT_set_state(IDLE);
        mCalibStep = CS_MOTOR_L_START;
        break;

    case CS_MOTOR_L_START:
        loop_count  = 0;
        Ialphas[0]  = 0.0f;                        // 初始化负电压方向的电流累积
        Ialphas[1]  = 0.0f;                        // 初始化正电压方向的电流累积
        voltages[0] = -UsrConfig.calib_voltage;    // 负测试电压
        voltages[1] = +UsrConfig.calib_voltage;    // 正测试电压
        FOC_voltage(voltages[0], 0, 0);            // 先施加负电压
        mCalibStep = CS_MOTOR_L_LOOP;
        break;

    case CS_MOTOR_L_LOOP: {
        int i = loop_count & 1;                    // 交替选择0或1(奇偶循环)
        Ialphas[i] += Foc.i_a;                     // 累积当前方向的电流值

        // Test voltage along phase A
        FOC_voltage(voltages[i], 0, 0);             // 交替施加正负电压
        // 完成预定周期后结束电感测量
        if (loop_count >= (num_L_cycles << 1)) {    // <<1相当于乘以2
            PWMC_TurnOnLowSides();                  // 停止电机
            mCalibStep = CS_MOTOR_L_END;
        }
    } break;

    case CS_MOTOR_L_END: {
        // 计算电流变化率: dI/dt = (I_neg - I_pos) / (时间)
        // 注意：这里用Ialphas[0]-Ialphas[1]确保dI/dt为正
        float dI_by_dt                   = (Ialphas[0] - Ialphas[1]) / (float) (CURRENT_MEASURE_PERIOD * num_L_cycles);
        float L                          = UsrConfig.calib_voltage / dI_by_dt;         // 计算电感: L = V / (dI/dt)
        UsrConfig.motor_phase_inductance = L * 2.0f / 3.0f;                            // 转换为相电感值
        FOC_update_current_ctrl_gain(UsrConfig.current_ctrl_bw);                       // 更新电流控制环增益

        uint8_t data[4];
        float_to_data(UsrConfig.motor_phase_inductance, data);
        CAN_calib_report(2, data);
        // 准备进入极对数识别
        phase_set  = 0;
        loop_count = 0;
        mCalibStep = CS_DIR_PP_START;
    } break;

    case CS_DIR_PP_START://方向与极对数识别
        FOC_voltage((voltage * time / 2.0f), 0, phase_set);// 斜坡增加电压，使电机缓慢启动
        if (time >= 2.0f) {                                // 2秒后开始正式测试
            start_count = (float) Encoder.shadow_count;    // 记录起始编码器位置
            mCalibStep  = CS_DIR_PP_LOOP;
            break;
        }
        break;

    case CS_DIR_PP_LOOP:
        phase_set += calib_phase_vel * CURRENT_MEASURE_PERIOD;// 以固定相位速度旋转   设置相
        FOC_voltage(voltage, 0, phase_set);
        if (phase_set >= 4.0f * M_2PI) {                      // 旋转4圈后结束
            mCalibStep = CS_DIR_PP_END;
        }
        break;

    case CS_DIR_PP_END: {
        int32_t diff = Encoder.shadow_count - start_count;    // 计算旋转的编码器脉冲数

        // Check direction
        if (diff > 0) {
            UsrConfig.encoder_dir = +1;
        } else {
            UsrConfig.encoder_dir = -1;
        }

        // Motor pole pairs  计算极对数: 4圈 / (编码器计数/每圈)
        UsrConfig.motor_pole_pairs = round(4.0f / ABS(diff / ENCODER_CPR_F));

        {
            uint8_t data[4];

            float_to_data((float) UsrConfig.motor_pole_pairs, data);
            CAN_calib_report(3, data);    // 报告极对数

            float_to_data((float) UsrConfig.encoder_dir, data);
            CAN_calib_report(4, data);    // 报告方向
        }

        mCalibStep = CS_ENCODER_START;
    } break;

    case CS_ENCODER_START://编码器偏移校准
        phase_set        = 0;
        loop_count       = 0;
        sample_count     = 0;
        next_sample_time = 0;

         // 提高编码器校准阶段的转速
        static const float encoder_calib_vel = 6.0f * M_PI;  // 专门用于编码器校准的速度

        mCalibStep       = CS_ENCODER_CW_LOOP;// 开始顺时针采样
        break;

    case CS_ENCODER_CW_LOOP:
        if (sample_count < (UsrConfig.motor_pole_pairs * SAMPLES_PER_PPAIR)) {// 顺时针采样：每个极对数采样SAMPLES_PER_PPAIR个点
            if (time > next_sample_time) {
                next_sample_time += M_2PI / ((float) SAMPLES_PER_PPAIR * encoder_calib_vel);

                int count_ref = (phase_set * ENCODER_CPR_F) / (M_2PI * (float) UsrConfig.motor_pole_pairs);   // 计算理论编码器位置(CPR)
                int error     = Encoder.raw - count_ref;                                                      // 计算编码器误差
                error += ENCODER_CPR * (error < 0);                                                           // 处理负误差
                p_error_arr[sample_count] = error;                                                            // 存储误差


                // ============ 新增：记录电流值 ============
                // 定义静态数组存储电流数据（在函数开头定义）
                
            
                // 记录当前A相电流值（来自GD30AP724的ADC采样）
                current_samples[sample_count] = Foc.i_a;
                
                ENCODERtest_sample[sample_count] = Encoder.raw;
                count_ref_sample[sample_count] = count_ref;


                sample_count++;
            }

            phase_set += encoder_calib_vel * CURRENT_MEASURE_PERIOD;                                            // 增加相位
        } else {
            phase_set -= encoder_calib_vel * CURRENT_MEASURE_PERIOD;                                            // 顺时针采样完成，准备逆时针采样
            loop_count = 0;
            sample_count--;
            next_sample_time = 0;
            mCalibStep       = CS_ENCODER_CCW_LOOP;
            break;
        }
        FOC_voltage(voltageencoder, 0, phase_set);
        break;

    case CS_ENCODER_CCW_LOOP:
        if (sample_count >= 0) {
            if (time > next_sample_time) {
                next_sample_time += M_2PI / ((float) SAMPLES_PER_PPAIR * encoder_calib_vel);

                int count_ref = (phase_set * ENCODER_CPR_F) / (M_2PI * (float) UsrConfig.motor_pole_pairs);
                int error     = Encoder.raw - count_ref;
                error += ENCODER_CPR * (error < 0);
                p_error_arr[sample_count] = (p_error_arr[sample_count] + error) / 2;                        // 取顺时针和逆时针采样的平均值

                sample_count--;
            }

            phase_set -= encoder_calib_vel * CURRENT_MEASURE_PERIOD;                                          // 减小相位
        } else {
            PWMC_TurnOnLowSides();
            mCalibStep = CS_ENCODER_END;
            break;
        }
        FOC_voltage(voltageencoder, 0, phase_set);
        break;

    case CS_ENCODER_END: {
        // Calculate average offset 计算平均编码器偏移
        int64_t moving_avg = 0;
        for (int i = 0; i < (UsrConfig.motor_pole_pairs * SAMPLES_PER_PPAIR); i++) {
            moving_avg += p_error_arr[i];
        }
        UsrConfig.encoder_offset = moving_avg / (UsrConfig.motor_pole_pairs * SAMPLES_PER_PPAIR);

        {
            uint8_t data[4];
            float_to_data((float) UsrConfig.encoder_offset, data);
            CAN_calib_report(5, data);// 报告偏移量
        }

        // FIR and map measurements to lut   创建偏移查找表(LUT)
        int window     = SAMPLES_PER_PPAIR;   // 滑动窗口大小
        int lut_offset = p_error_arr[0] * OFFSET_LUT_NUM / ENCODER_CPR;
        for (int i = 0; i < OFFSET_LUT_NUM; i++) {
            moving_avg = 0;
            for (int j = (-window) / 2; j < (window) / 2; j++) {                       // 滑动窗口平均滤波
                int index = i * UsrConfig.motor_pole_pairs * SAMPLES_PER_PPAIR / OFFSET_LUT_NUM + j;
                if (index < 0) {                                                       // 处理边界条件
                    index += (SAMPLES_PER_PPAIR * UsrConfig.motor_pole_pairs);
                } else if (index > (SAMPLES_PER_PPAIR * UsrConfig.motor_pole_pairs - 1)) {
                    index -= (SAMPLES_PER_PPAIR * UsrConfig.motor_pole_pairs);
                }
                moving_avg += p_error_arr[index];
            }
            moving_avg    = moving_avg / window;
            int lut_index = lut_offset + i;// 计算LUT索引并存储
            if (lut_index > (OFFSET_LUT_NUM - 1)) {
                lut_index -= OFFSET_LUT_NUM;
            }
            UsrConfig.offset_lut[lut_index] = moving_avg - UsrConfig.encoder_offset;
        }
        // 准备报告LUT
        loop_count       = 0;
        sample_count     = 0;
        next_sample_time = 0;
        mCalibStep       = CS_REPORT_OFFSET_LUT;
    } break;

    case CS_REPORT_OFFSET_LUT:// 通过CAN总线逐个报告LUT值
        if (sample_count < OFFSET_LUT_NUM) {
            if (time > next_sample_time) {
                next_sample_time += 0.001f;
                {
                    uint8_t data[4];
                    float_to_data((float) UsrConfig.offset_lut[sample_count], data);
                    CAN_calib_report(10 + sample_count, data);
                }
                sample_count++;
            }
        } else {
            mCalibStep            = CS_NULL;
            UsrConfig.calib_valid = true;
            MCT_set_state(IDLE);
        }
        break;

    default:
        break;
    }

    loop_count++;
}
