#include "rgb.h"
#include <string.h>
#include "gd32g5x3_init.h"
#include "main.h"
#define SYSTEM_CLOCK_FREQ 216000000  // 216MHz

// 基于216MHz的精确纳秒延时
void delay_ns_216mhz(uint32_t ns)
{
    // 216MHz下，1个时钟周期 ≈ 4.63ns
    // 计算需要的循环次数（每条指令约2-4个时钟周期）
    volatile uint32_t cycles = (ns * SYSTEM_CLOCK_FREQ) / 1000000000 / 3;
    while(cycles--);
}

// 精确的微秒延时
void delay_us_216mhz(uint32_t us)
{
    volatile uint32_t cycles = us * (SYSTEM_CLOCK_FREQ / 1000000) / 3;
    while(cycles--);
}


rgb_color_t rgb_buffer[LED_COUNT];

// 延时函数（需要根据CPU频率精确调整）
void ws2812b_delay_ns(uint32_t ns)
{
    volatile uint32_t delay = (ns * SYSTEM_CLOCK_FREQ) / 1000000000 / 3;;
    while(delay--);
}

// 发送一个比特
void ws2812b_send_bit(uint8_t bit)
{
    if(bit) {
        // 发送'1'：高电平0.8μs + 低电平0.45μs
        gpio_bit_set(WS2812B_PORT, WS2812B_PIN);
        ws2812b_delay_ns(700);  // 0.8μs高电平
        gpio_bit_reset(WS2812B_PORT, WS2812B_PIN);
        ws2812b_delay_ns(600);  // 0.45μs低电平
    } else {
        // 发送'0'：高电平0.4μs + 低电平0.85μs
        gpio_bit_set(WS2812B_PORT, WS2812B_PIN);
        ws2812b_delay_ns(350);  // 0.4μs高电平
        gpio_bit_reset(WS2812B_PORT, WS2812B_PIN);
        ws2812b_delay_ns(800);  // 0.85μs低电平
    }
}

// 发送一个字节（MSB first）
void ws2812b_send_byte(uint8_t byte)
{
    for(int i = 7; i >= 0; i--) {
        ws2812b_send_bit((byte >> i) & 0x01);
    }
}

// 发送一个LED的颜色数据
void ws2812b_send_color(rgb_color_t color)
{
    ws2812b_send_byte(color.g);  // WS2812B顺序是GRB
    ws2812b_send_byte(color.r);
    ws2812b_send_byte(color.b);
}

// 发送复位信号
void ws2812b_reset(void)
{
    gpio_bit_reset(WS2812B_PORT, WS2812B_PIN);
    // 延时超过50μs
    for(volatile int i = 0; i < 1000; i++);  // 需要根据实际时钟调整
}

// 初始化WS2812B GPIO
void ws2812b_init(void)
{
    /* 配置引脚为推挽输出，高速模式 */
    rcu_periph_clock_enable(RCU_GPIOB);
    gpio_mode_set(WS2812B_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, WS2812B_PIN);
    gpio_output_options_set(WS2812B_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100_220MHZ, WS2812B_PIN);
    
    // 初始化为低电平
    gpio_bit_reset(WS2812B_PORT, WS2812B_PIN);
}

// 设置单个LED颜色
void ws2812b_set_led(uint8_t led_index, rgb_color_t color, rgb_color_t *led_buffer)
{
    if(led_index < 2) {
        led_buffer[led_index] = color;
    }
}

// 设置所有LED为同一颜色
void ws2812b_set_all(rgb_color_t color, rgb_color_t *led_buffer, uint16_t led_count)
{
    for(int i = 0; i < led_count; i++) {
        led_buffer[i] = color;
    }
}

// 更新所有LED（发送数据到WS2812B）
void ws2812b_update(rgb_color_t *led_buffer, uint16_t led_count)
{
    for(int i = 0; i < led_count; i++) {
        ws2812b_send_color(led_buffer[i]);
    }
    ws2812b_reset();
}

// HSV到RGB转换
void hsv_to_rgb(uint8_t h, uint8_t s, uint8_t v, rgb_color_t *color)
{
    uint8_t region, remainder, p, q, t;
    
    if(s == 0) {
        color->r = color->g = color->b = v;
        return;
    }
    
    region = h / 43;
    remainder = (h - (region * 43)) * 6;
    
    p = (v * (255 - s)) >> 8;
    q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;
    
    switch(region) {
        case 0:  color->r = v; color->g = t; color->b = p; break;
        case 1:  color->r = q; color->g = v; color->b = p; break;
        case 2:  color->r = p; color->g = v; color->b = t; break;
        case 3:  color->r = p; color->g = q; color->b = v; break;
        case 4:  color->r = t; color->g = p; color->b = v; break;
        default: color->r = v; color->g = p; color->b = q; break;
    }
}
// 预定义颜色
const rgb_color_t COLOR_RED    = {0, 255, 0};
const rgb_color_t COLOR_GREEN  = {255, 0, 0};
const rgb_color_t COLOR_BLUE   = {0, 0, 255};
const rgb_color_t COLOR_WHITE  = {255, 255, 255};
const rgb_color_t COLOR_YELLOW = {255, 255, 0};
const rgb_color_t COLOR_PURPLE = {0, 128, 255};
const rgb_color_t COLOR_ORANGE = {0, 165, 255};
const rgb_color_t COLOR_OFF    = {0, 0, 0};

// 彩虹效果
void rainbow_effect(rgb_color_t *led_buffer, uint16_t led_count, uint8_t hue_offset)
{
    for(int i = 0; i < led_count; i++) {
        uint8_t hue = (i * 255 / led_count + hue_offset) % 255;
        hsv_to_rgb(hue, 255, 255, &led_buffer[i]);
    }
}

// 呼吸灯效果
void breathing_effect(rgb_color_t *led_buffer, uint16_t led_count, rgb_color_t color, uint8_t brightness)
{
    rgb_color_t dimmed_color;
    dimmed_color.r = (color.r * brightness) / 255;
    dimmed_color.g = (color.g * brightness) / 255; 
    dimmed_color.b = (color.b * brightness) / 255;
    
    ws2812b_set_all(dimmed_color, led_buffer, led_count);
}

// 流水灯效果
void running_light(rgb_color_t *led_buffer, uint16_t led_count, rgb_color_t color, uint8_t position)
{
    ws2812b_set_all(COLOR_OFF, led_buffer, led_count);
    
    // 设置多个LED亮起，形成流动效果
    for(int i = 0; i < 3; i++) {
        uint8_t led_pos = (position + i) % led_count;
        ws2812b_set_led(led_pos, color, led_buffer);
    }
}

void simple_rainbow_demo(void)
{
    uint8_t hue_offset = 0;
    
    while(1) {
        rainbow_effect(rgb_buffer, LED_COUNT, hue_offset);
        ws2812b_update(rgb_buffer, LED_COUNT);
        hue_offset += 5;  // 调整这个值可以改变流动速度
        delay_ms(100);
    }
}

// 单个灯珠HSV色调渐变
void single_led_hsv_gradient(void)
{
    rgb_color_t led_buffer[1];
    uint8_t hue = 0;     // 色调 (0-255)
    uint8_t saturation = 255;  // 饱和度 (0-255)
    uint8_t value = 255;       // 亮度 (0-255)
    
    while(1) {
        // HSV转RGB
        hsv_to_rgb(hue, saturation, value, &led_buffer[0]);
        
        // 更新LED
        ws2812b_update(led_buffer, 1);
        
        // 色调递增，实现彩虹效果
        hue++;
        
        delay_ms(30);  // 控制渐变速度
    }
}

void single_led_color_cycle(void)
{
    rgb_color_t led_buffer[1];  // 单个LED的缓冲区
    uint8_t r = 255, g = 0, b = 0;  // 从红色开始
    
    while(1) {
        // 设置单个LED颜色
        led_buffer[0] = (rgb_color_t){g, r, b};  // GRB顺序
        
        // 更新到WS2812B
        ws2812b_update(led_buffer, 1);
        
        // 颜色渐变逻辑
        if(r == 255 && g < 255 && b == 0) g++;      // 红→黄
        else if(g == 255 && r > 0 && b == 0) r--;   // 黄→绿
        else if(g == 255 && b < 255 && r == 0) b++; // 绿→青
        else if(b == 255 && g > 0 && r == 0) g--;   // 青→蓝
        else if(b == 255 && r < 255 && g == 0) r++; // 蓝→紫
        else if(r == 255 && b > 0 && g == 0) b--;   // 紫→红
        
        delay_ms(1000);  // 控制渐变速度
    }
}


void debug_gradient_step_by_step(void)
{
    rgb_color_t led_buffer[1];
    
    ws2812b_init();
    
    // 步骤1：测试基本颜色显示
    //printf("测试红色...\n");
    led_buffer[0] = (rgb_color_t){0, 255, 0};  // 红色(GRB)
    ws2812b_update(led_buffer, 1);
    delay_ms(2000);
    
    //printf("测试绿色...\n");
    led_buffer[0] = (rgb_color_t){255, 0, 0};   // 绿色
    ws2812b_update(led_buffer, 1);
    delay_ms(2000);
    
    //printf("测试蓝色...\n");
    led_buffer[0] = (rgb_color_t){0, 0, 255};   // 蓝色
    ws2812b_update(led_buffer, 1);
    delay_ms(2000);
    
    // 步骤2：测试HSV转换
    //printf("测试HSV转换...\n");
    for(uint8_t hue = 0; hue < 255; hue += 10) {
        hsv_to_rgb(hue, 255, 255, &led_buffer[0]);
        ws2812b_update(led_buffer, 1);
        //printf("Hue: %d -> R:%d G:%d B:%d\n", 
               //hue, led_buffer[0].r, led_buffer[0].g, led_buffer[0].b);
        delay_ms(500);
    }
    
    // 步骤3：完整渐变
    //printf("开始完整渐变...\n");
    uint8_t hue = 0;
    while(1) {
        hsv_to_rgb(hue, 255, 255, &led_buffer[0]);
        ws2812b_update(led_buffer, 1);
        hue++;
        delay_ms(50);
    }
}


void debug_gradient(void)
{
    rgb_color_t led_buffer[1];
    uint8_t hue = 0;
    
    while(1) {
        // 添加调试输出
        //printf("Hue value: %d\n", hue);  // 检查hue是否在变化
        
        hsv_to_rgb(0, 0, 0, &led_buffer[0]);
        ws2812b_update(led_buffer, 1);
        
        hue++;  // 确保这个递增操作在执行
        
        // 使用简单的延时，避免复杂的延时函数问题
        for(int i = 0; i < 100000; i++);  // 简单循环延时
    }
}