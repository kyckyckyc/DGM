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

#include "encoder.h"
#include "pwm_curr.h"
#include "usr_config.h"
#include "util.h"
extern     spi_parameter_struct spi0_parameter;

tEncoder Encoder;

void ENCODER_init(void)
{
    // Init
    Encoder.need_init    = 20;
    Encoder.shadow_count = 0;
    Encoder.pll_pos      = 0;
    Encoder.pll_vel      = 0;

    int encoder_pll_bw     = 100 * M_2PI;
    Encoder.pll_kp         = 2.0f * encoder_pll_bw;      // basic conversion to discrete time
    Encoder.pll_ki         = 0.25f * SQ(Encoder.pll_kp); // Critically damped
    Encoder.snap_threshold = 0.5f * CURRENT_MEASURE_PERIOD * Encoder.pll_ki;
}

static inline void delay_100ns(void)
{
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
}
 
uint16_t u16Data = 0x0000;
int32_t ENCODER_read(void)
{
  
 
    uint16_t u16Command = 0xFFFF;
    
    uint16_t u16Safe = 0x0000;
 
    u16Command = READ_ANGLE_VALUE;
    u16Safe = cdd_TLI5012OUT_ReadReg(u16Command, &u16Data);
    u16Data = u16Data & 0x7FFF;

    return (u16Data>>1 );
}
 

void ENCODER_loop(void)
{
    Encoder.raw = ENCODER_read();
    if (UsrConfig.encoder_dir == -1) {
        Encoder.raw = ENCODER_CPR - 1 - Encoder.raw;
    }

    /* Linearization */
    int off_1      = UsrConfig.offset_lut[(Encoder.raw) >> 7];
    int off_2      = UsrConfig.offset_lut[((Encoder.raw >> 7) + 1) % 128];
    int off_interp = off_1 + ((off_2 - off_1) * (Encoder.raw - ((Encoder.raw >> 7) << 7)) >> 7);

    int count = Encoder.raw - off_interp - UsrConfig.encoder_offset;

    /*  Wrap in ENCODER_CPR */
    while (count > ENCODER_CPR)
        count -= ENCODER_CPR;
    while (count < 0)
        count += ENCODER_CPR;

    Encoder.count_in_cpr = count;

    if (Encoder.need_init) {
        Encoder.need_init--;
        Encoder.count_in_cpr_prev = Encoder.count_in_cpr;
        return;
    }

    /* Delta count */
    int delta_count           = Encoder.count_in_cpr - Encoder.count_in_cpr_prev;
    Encoder.count_in_cpr_prev = Encoder.count_in_cpr;
    while (delta_count > +ENCODER_CPR_DIV)
        delta_count -= ENCODER_CPR;
    while (delta_count < -ENCODER_CPR_DIV)
        delta_count += ENCODER_CPR;

    // Run pll (for now pll is in units of encoder counts)
    // Predict current pos
    Encoder.pll_pos += CURRENT_MEASURE_PERIOD * Encoder.pll_vel;
    // Discrete phase detector
    float delta_pos = Encoder.count_in_cpr - floorf(Encoder.pll_pos);
    while (delta_pos > +ENCODER_CPR_DIV)
        delta_pos -= ENCODER_CPR_F;
    while (delta_pos < -ENCODER_CPR_DIV)
        delta_pos += ENCODER_CPR_F;
    // PLL feedback
    Encoder.pll_pos += CURRENT_MEASURE_PERIOD * Encoder.pll_kp * delta_pos;
    while (Encoder.pll_pos > ENCODER_CPR)
        Encoder.pll_pos -= ENCODER_CPR_F;
    while (Encoder.pll_pos < 0)
        Encoder.pll_pos += ENCODER_CPR_F;
    Encoder.pll_vel += CURRENT_MEASURE_PERIOD * Encoder.pll_ki * delta_pos;

    // Align delta-sigma on zero to prevent jitter
    if (ABS(Encoder.pll_vel) < Encoder.snap_threshold) {
        Encoder.pll_vel = 0.0f;
    }

    /* Outputs from Encoder for Controller */
    Encoder.shadow_count += delta_count;
    Encoder.pos       = Encoder.shadow_count / ENCODER_CPR_F;
    Encoder.vel       = Encoder.pll_vel / ENCODER_CPR_F;
    Encoder.phase     = (M_2PI * UsrConfig.motor_pole_pairs) * Encoder.count_in_cpr / ENCODER_CPR_F;
    Encoder.phase_vel = (M_2PI * UsrConfig.motor_pole_pairs) * Encoder.vel;
}


uint16_t spi_rw_half_word(uint32_t spi_periph, const uint16_t i_HalfWord)
{
    uint32_t to;
    uint16_t data = 0xFFFF;
    bool ok = true;

    // Send data
    to = 200;
    while ((RESET == spi_flag_get(spi_periph, SPI_FLAG_TBE)) && --to);
    if (to == 0) ok = false;

    if (ok) {
        if(RESET == spi_flag_get(spi_periph, SPI_FLAG_RBNE)) {
            (void)spi_data_receive(spi_periph); /* throw Dummy */
        }
        
        spi_data_transmit(spi_periph, i_HalfWord);

        /* wait rx succ*/
        to = 200;
        while ((RESET == spi_flag_get(spi_periph, SPI_FLAG_RBNE)) && --to);
        if (to == 0) ok = false;
        else data = spi_data_receive(spi_periph); /* throw Dummy wait bsu idle */

        if (ok) {
            to = 200;
            while ((SET == spi_flag_get(spi_periph, SPI_FLAG_TRANS)) 
                   && SET == spi_flag_get(spi_periph, SPI_FLAG_RBNE) && --to);
            if (to == 0) ok = false;
        }
    }

    return ok ? data : 0xFFFF;
}


void cdd_TLI5012_Init(void)
{
	uint16_t u16Command = 0xFFFF;
	uint16_t u16Data = 0x0000;
	uint16_t u16Safe = 0x0000;
	
	/* ---------- Step 1: Read and configure STAT register ---------- */
	/* Read STAT register to get current status */
	u16Command = READ_STAT_VALUE;
	u16Safe = cdd_TLI5012_ReadReg(u16Command, &u16Data);
	 
	
	/* Write STAT register, set Slave_Number = 00 */
	u16Command = 0x0001;
	u16Data &= 0x9FFF;  /* Clear bits 13-14 (Slave_Number) */
	u16Safe = cdd_TLI5012_WriteReg(u16Command, u16Data);
	 
	
	/* Read STAT register again to verify write operation */
	u16Command = READ_STAT_VALUE;
	u16Safe = cdd_TLI5012_ReadReg(u16Command, &u16Data);
	 
	/* ---------- Step 2: Read ACSTAT register ---------- */
	u16Command = READ_ACSTAT_VALUE;
	u16Safe = cdd_TLI5012_ReadReg(u16Command, &u16Data);
	 
	
	/* ---------- Step 3: Configure MOD1 register ---------- */
	/* Read current MOD1 register value */
	u16Command = RD_REG | WRITE_MOD1_VALUE;
	u16Safe = cdd_TLI5012_ReadReg(u16Command, &u16Data);
	 
	
	/* Write MOD1 register with configuration value */
	u16Command = WRITE_MOD1_VALUE;
	u16Data = MOD1_VALUE;
	u16Safe = cdd_TLI5012_WriteReg(u16Command, u16Data);
	 
	
	/* Verify MOD1 register write operation */
	u16Command = RD_REG | WRITE_MOD1_VALUE;
	u16Safe = cdd_TLI5012_ReadReg(u16Command, &u16Data);
	 
	
	/* ---------- Step 4: Configure MOD2 register ---------- */
	/* Read current MOD2 register value */
	u16Command = RD_REG | WRITE_MOD2_VALUE;
	u16Safe = cdd_TLI5012_ReadReg(u16Command, &u16Data);
	 
	
	/* Write MOD2 register with configuration value */
	u16Command = WRITE_MOD2_VALUE;
	u16Data = MOD2_VALUE;
	u16Safe = cdd_TLI5012_WriteReg(u16Command, u16Data);
	 
	
	/* Verify MOD2 register write operation */
	u16Command = RD_REG | WRITE_MOD2_VALUE;
	u16Safe = cdd_TLI5012_ReadReg(u16Command, &u16Data);
	 

	/* ---------- Step 5: Configure MOD3 register ---------- */
	/* Read current MOD3 register value */
	u16Command = RD_REG | WRITE_MOD3_VALUE;
	u16Safe = cdd_TLI5012_ReadReg(u16Command, &u16Data);
	 
	
	/* Write MOD3 register with configuration value */
	u16Command = WRITE_MOD3_VALUE;
	u16Data = MOD3_VALUE;
	u16Safe = cdd_TLI5012_WriteReg(u16Command, u16Data);
	 
	
	/* Verify MOD3 register write operation */
	u16Command = RD_REG | WRITE_MOD3_VALUE;
	u16Safe = cdd_TLI5012_ReadReg(u16Command, &u16Data);
	 
	
	/* Optional: Configure MOD3 register with SSC_OD = 1 (commented out) */
	/*
	u16Command = 0x5091;
	u16Data |= 0x0004;  // Set SSC_OD bit
	u16Safe = cdd_TLI5012_WriteReg(u16Command, u16Data);
	printf("\r\n @Joe [TLI5012Init] [MOD3] write u16Data: 0x%04X, SafeWord: 0x%04X \r\n", u16Data, u16Safe);
	*/
	
	/* ---------- Step 6: Configure MOD4 register ---------- */
	/* Read current MOD4 register value */
	u16Command = RD_REG | WRITE_MOD4_VALUE;
	u16Safe = cdd_TLI5012_ReadReg(u16Command, &u16Data);
	 
	
	/* Write MOD4 register with configuration value */
	u16Command = WRITE_MOD4_VALUE;
	u16Data = MOD4_VALUE;
	u16Safe = cdd_TLI5012_WriteReg(u16Command, u16Data);
	 
	
	/* Verify MOD4 register write operation */
	u16Command = RD_REG | WRITE_MOD4_VALUE;
	u16Safe = cdd_TLI5012_ReadReg(u16Command, &u16Data);
	 
}

/**
 * @brief Write data to TLI5012B register
 * @param i_Cmd Command word to send
 * @param i_Data Data to write to register
 * @return Register response value, 0xFFFF if error
 * @note This function performs a complete SPI write transaction:
 */
uint16_t cdd_TLI5012_WriteReg(uint16_t i_Cmd, uint16_t i_Data)
{
    uint16_t result = 0xFFFF; /* Default error code */
    SPI_CS_ENABLE();
    
    result = spi_rw_half_word(SPI0, i_Cmd);
    result = spi_rw_half_word(SPI0, i_Data);
    
    SPI_TX_OFF(); /* Configure MOSI as input to avoid conflicts */
    result = spi_rw_half_word(SPI0, DUMMY_BYTE);
    SPI_CS_DISABLE();
    SPI_TX_ON(); /* Restore MOSI as output */
    
    return result;
}


/**
 * @brief Read data from TLI5012B register
 * @param i_Cmd Command word to send
 * @param i_Data Not used in read operation (kept for compatibility)
 * @return Register data value, 0xFFFF if error
 * @note This function performs a complete SPI read transaction:
 */
uint16_t cdd_TLI5012_ReadReg(uint16_t i_Cmd, uint16_t * const  i_Data)
{
    uint16_t result = 0xFFFF; /* Default error code */
    SPI_CS_ENABLE();
    
    *i_Data = spi_rw_half_word(SPI0, i_Cmd);
    SPI_TX_OFF(); /* Configure MOSI as input to avoid conflicts */
    *i_Data = spi_rw_half_word(SPI0, DUMMY_BYTE);
    result = spi_rw_half_word(SPI0, DUMMY_BYTE);
    
    SPI_CS_DISABLE();
    SPI_TX_ON(); /* Restore MOSI as output */
    
    return result;
}

uint16_t cdd_TLI5012OUT_ReadReg(uint16_t i_Cmd, uint16_t * const  i_Data)
{
    uint16_t result = 0xFFFF; /* Default error code */
    SPI_CS1_ENABLE();
    
    *i_Data = spi_rw_half_word(SPI0, i_Cmd);
    SPI_TX_OFF(); /* Configure MOSI as input to avoid conflicts */
    *i_Data = spi_rw_half_word(SPI0, DUMMY_BYTE);
    result = spi_rw_half_word(SPI0, DUMMY_BYTE);
    
    SPI_CS1_DISABLE();
    SPI_TX_ON(); /* Restore MOSI as output */
    
    return result;
}


void Test_TLI5012B(void)
{
    uint16_t u16Command = 0xFFFF;
    uint16_t u16Data = 0x0000;
    uint16_t u16Safe = 0x0000;
    float angle = 0;

    u16Command = READ_ANGLE_VALUE;
    u16Safe = cdd_TLI5012_ReadReg(u16Command, &u16Data);
    u16Data = u16Data & 0x7FFF;
    
     
    angle = (360 * (((int16_t)(u16Data << 1)) >> 1)) >> 15;
 }


void Test_TLI5012BOUT(void)
{
    uint16_t u16Command = 0xFFFF;
    uint16_t u16Data = 0x0000;
    uint16_t u16Safe = 0x0000;
    float angle = 0;

    u16Command = READ_ANGLE_VALUE;
    u16Safe = cdd_TLI5012OUT_ReadReg(u16Command, &u16Data);
    u16Data = u16Data & 0x7FFF;
    
     
    angle = (360 * (((int16_t)(u16Data << 1)) >> 1)) >> 15;
 }