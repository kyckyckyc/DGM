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

#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"

// #define ENCODER_CPR     (int) 16384
// #define ENCODER_CPR_F   (16384.0f)
// #define ENCODER_CPR_DIV (ENCODER_CPR >> 1)


// TLE5012B 是16位绝对编码器，每转计数为65536



#define ENCODER_NATIVE_OFFSET 32768   // TLE5012B零位对应的原生值
#define ENCODER_NATIVE_MAX    65536   // TLE5012B最大原生值
#define ENCODER_CPR     (int)32768        // 16位分辨率
#define ENCODER_CPR_F   (32768.0f)        // 浮点版本
#define ENCODER_CPR_DIV (ENCODER_CPR >> 1) // 32768，用于半转计算



typedef struct sEncoder
{
    uint8_t need_init;

    int raw;
    int count_in_cpr;
    int count_in_cpr_prev;

    int64_t shadow_count;

    float pos;
    float vel;
    float phase;
    float phase_vel;

    // pll use
    float pll_pos;
    float pll_vel;

    float pll_kp;
    float pll_ki;
    float interpolation;
    float snap_threshold;
} tEncoder;

extern tEncoder Encoder;

void ENCODER_init(void);
bool ENCODER_sample(void);
void ENCODER_loop(void);
int32_t ENCODER_read(void);

//uint16_t spi_rw_half_word(uint32_t spi_periph, const uint16_t i_HalfWord); 
 
 // =============================================
// 手动选择编码器类型（取消注释其中一个）
// =============================================

// 选择KTH71编码器（取消注释下面这行）
//#define KTH71 

// 选择TLE5012B编码器（取消注释下面这行） 
#define TLE5012B 

// 默认配置（如果没有定义任何编码器，使用KTH71）
#if !defined(KTH71) && !defined(TLE5012B)
    #warning "No encoder defined, defaulting to KTH71"
    #define KTH71
#endif

// 防止同时定义多个编码器
#if defined(KTH71) && defined(TLE5012B)
    #error "Cannot define both KTH71 and TLE5012B at the same time"
#endif 


/**
 * @brief SPI command definitions for TLI5012B sensor
 * @note All commands are 16-bit values following the TLI5012B protocol
 */

/* SPI operation types */
#define WRITE			  0 /**< SPI Write Operation */
#define READ			  1 /**< SPI Read Operation */

/* SPI communication constants */
#define DUMMY_BYTE		  0xFFFF /**< Dummy byte for SPI read operations */

#define WR_REG			  0x0000 /**< Write command - MSB is 0 */
#define RD_REG			  0x8000 /**< Read command - MSB is 1 */

#define LockValue_LADDR	  0x0000 /**< Lock value for addresses 0x00-0x04 */
#define LockValue_HADDR	  0x5000 /**< Lock value for addresses 0x05-0x11 */

/* Update buffer access command */
#define UPD_CMD			  0x0400 /**< Update command - bit 10 is 1, access update buffer */

/* Read command values */
#define READ_STAT_VALUE	  0x8001 /**< Read status register */
#define READ_ACSTAT_VALUE 0x8011 /**< Read activation status register */

#if defined(TLE5012B)
#define READ_ANGLE_VALUE  0x8021 /**< Read angle value register */
#elif defined(KTH71)
#define READ_ANGLE_VALUE  0x0000 /**< Read angle value register */
#endif

#define READ_SPEED_VALUE  0x8031 /**< Read speed value register */

/* Write command values and their corresponding data */
#define WRITE_MOD1_VALUE  0x5061 /**< Write to MOD1 register (0_1010_0_000110_0001) */
#define MOD1_VALUE		  0x0001 /**< MOD1 register configuration value */

#define WRITE_MOD2_VALUE  0x5081 /**< Write to MOD2 register (0_1010_0_001000_0001) */
#define MOD2_VALUE		  0x0809 /**< MOD2 register configuration value */

#define WRITE_MOD3_VALUE  0x5091 /**< Write to MOD3 register (0_1010_0_001001_0001) */
#define MOD3_VALUE		  0x0000 /**< MOD3 register configuration value */

#define WRITE_MOD4_VALUE  0x50E1 /**< Write to MOD4 register (0_1010_0_001110_0001) */
#define MOD4_VALUE                                                               \
	0x0080 /**< MOD4 register value: 12-bit 4096, absolute count disable 0x0080; \
			enable 0x0000 */

#define WRITE_IFAB_VALUE 0x50B1 /**< Write to IFAB register */
#define IFAB_VALUE		 0x000D /**< IFAB register configuration value */

/**
 * @brief TLE5012 register address definitions
 * @note Register addresses are 4-bit values (0x00-0x11, 0x20)
 */
/* Status and data registers (0x00-0x04) */
#define STAT_ADDR		 0x00 /**< STATus register */
#define ACSTAT_ADDR		 0x01 /**< ACtivation STATus register */
#define AVAL_ADDR		 0x02 /**< Angle VALue register */
#define ASPD_ADDR		 0x03 /**< Angle SPeeD register */
#define AREV_ADDR		 0x04 /**< Angle REVolution register */

/* Configuration registers (0x05-0x0E) */
#define FSYNC_ADDR		 0x05 /**< Frame SYNChronization register */
#define MOD_1_ADDR		 0x06 /**< Interface MODe1 register */
#define SIL_ADDR		 0x07 /**< SIL register */
#define MOD_2_ADDR		 0x08 /**< Interface MODe2 register */
#define MOD_3_ADDR		 0x09 /**< Interface MODe3 register */
#define OFFX_ADDR		 0x0A /**< OFFset X register */
#define OFFY_ADDR		 0x0B /**< OFFset Y register */
#define SYNCH_ADDR		 0x0C /**< SYNCHronicity register */
#define IFAB_ADDR		 0x0D /**< IFAB register */
#define MOD_4_ADDR		 0x0E /**< Interface MODe4 register */

/* Additional registers (0x0F-0x11, 0x20) */
#define TCO_Y_ADDR		 0x0F /**< Temperature COefficient register */
#define ADC_X_ADDR		 0x10 /**< ADC X-raw value register */
#define ADC_Y_ADDR		 0x11 /**< ADC Y-raw value register */
#define IIF_CNT_ADDR	 0x20 /**< IIF CouNTer value register */




#define SPI_TX_OFF() \
do { gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_7);} while(0);

#define SPI_TX_ON() \
do { gpio_af_set(GPIOA, GPIO_AF_5, GPIO_PIN_7);gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, GPIO_PIN_7);} while(0);


#define SPI_CS_ENABLE()       ENC_NCS_RESET();
#define SPI_CS_DISABLE()       ENC_NCS_SET();

#define SPI_CS1_ENABLE()       ENC_NCS1_RESET();
#define SPI_CS1_DISABLE()       ENC_NCS1_SET();

uint16_t spi_rw_half_word(uint32_t spi_periph, const uint16_t i_HalfWord);
uint16_t cdd_TLI5012_ReadReg(uint16_t i_Cmd, uint16_t * const  i_Data);
uint16_t cdd_TLI5012_WriteReg(uint16_t i_Cmd, uint16_t i_Data);
uint16_t cdd_TLI5012OUT_ReadReg(uint16_t i_Cmd, uint16_t * const  i_Data);


void cdd_TLI5012_Init(void);

void Test_TLI5012B(void);
void Test_TLI5012BOUT(void);
 
 




#endif
