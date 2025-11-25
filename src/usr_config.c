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

#include "usr_config.h"
#include "controller.h"
#include "heap.h"
#include "util.h"
#include <string.h>

tUsrConfig   UsrConfig;
tCoggingMap *pCoggingMap = NULL;

void USR_CONFIG_set_default_config(void)
{
    // Motor
    UsrConfig.invert_motor_dir       = 0;
    UsrConfig.motor_pole_pairs       = 21;
    UsrConfig.motor_phase_resistance = 0.07685f;
    UsrConfig.motor_phase_inductance = 18e-6f;
    UsrConfig.current_limit          = 5;
    UsrConfig.velocity_limit         = 60;

    // Encoder
    UsrConfig.calib_current = 1.5f;
    UsrConfig.calib_voltage = 5.0f;

    // Controller
    UsrConfig.pos_p_gain             = 80.0f;
    UsrConfig.vel_p_gain             = 0.3f;
    UsrConfig.vel_i_gain             = 5.0f;
    UsrConfig.current_ff_gain        = 0.001f;
    UsrConfig.current_ctrl_bw        = 1000;
    UsrConfig.default_op_mode        = CONTROL_MODE_POSITION_PROFILE;
    UsrConfig.anticogging_enable     = 1;
    UsrConfig.sync_target_enable     = 0;
    UsrConfig.target_velcity_window  = 0.5f;
    UsrConfig.target_position_window = 0.01f;
    UsrConfig.current_ramp_rate      = 0.5f;
    UsrConfig.velocity_ramp_rate     = 50;
    UsrConfig.position_filter_bw     = 10;
    UsrConfig.profile_velocity       = 50;
    UsrConfig.profile_accel          = 50;
    UsrConfig.profile_decel          = 50;

    // Protect
    UsrConfig.protect_under_voltage = 12;
    UsrConfig.protect_over_voltage  = 50;
    UsrConfig.protect_over_current  = 10;
    UsrConfig.protect_drv_over_tmp  = 80;
    UsrConfig.protect_ntc_over_tmp  = 80;

    // CAN
    UsrConfig.node_id               = 1;
    UsrConfig.can_baudrate          = CAN_BAUDRATE_500K;
    UsrConfig.heartbeat_consumer_ms = 0;
    UsrConfig.heartbeat_producer_ms = 0;

    // Encoder
    UsrConfig.calib_valid = 0;
}

int USR_CONFIG_erease_config(void)
{
    uint32_t       addr;
    fmc_state_enum status;
    uint32_t page_number;

    fmc_unlock();


    // Erase
    for (addr = USR_CONFIG_ADDR; addr < (USR_CONFIG_ADDR + USR_CONFIG_MAX_SIZE); addr += PAGE_SIZE) {
         page_number = (addr - MAIN_FLASH_BASE_ADDRESS) / PAGE_SIZE;
        fmc_flag_clear(FMC_FLAG_ENDF | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);
        status = fmc_page_erase(FMC_BANK0,page_number);
        if (status != FMC_READY) {
            fmc_lock();
            return -1;
        }
    }

    fmc_lock();

    // Check
    for (addr = USR_CONFIG_ADDR; addr < (USR_CONFIG_ADDR + USR_CONFIG_MAX_SIZE); addr += 4) {
        if (0xFFFFFFFF != *((uint32_t *) addr)) {
            return -2;
        }
    }

    return 0;
}

int USR_CONFIG_read_config(void)
{
    int state = 0;

    memcpy(&UsrConfig, (uint8_t *) USR_CONFIG_ADDR, sizeof(tUsrConfig));

    uint32_t crc;
    crc = crc32((uint8_t *) &UsrConfig, sizeof(tUsrConfig) - 4);
    if (crc != UsrConfig.crc) {
        state = -1;
    }

    return state;
}

uint32_t savecnt=0;
uint32_t ram_value=0,ram_value1=0;
uint64_t flashdata = 0;
uint32_t flash_address=0;
uint64_t testfmcdata = 0;
uint32_t length = 0;
int USR_CONFIG_save_config(void)
{
    fmc_state_enum fmc_state = FMC_READY;
    
    // Step 1: Erase flash first
    if (USR_CONFIG_erease_config()) {
        return -1;
    }

    // Step 2: Unlock flash controller
    fmc_unlock();
    length =  sizeof(tUsrConfig)/8;
    // Step 3: Calculate CRC before programming
    UsrConfig.crc = crc32((uint8_t *)&UsrConfig, sizeof(tUsrConfig) - 4);
    
    // Step 4: Program data word by word
    uint32_t *pData = (uint32_t *)&UsrConfig;
    
    for (int i = 0; i < sizeof(tUsrConfig) / 8 + 1; i++) {
        if(i<length)
        {
            ram_value = *(pData + 2*i);
            ram_value1 = *(pData + 2*i + 1);
            flashdata = ((uint64_t)ram_value1 << 32) | ram_value;
            flash_address = USR_CONFIG_ADDR + i * 8;
        }
        if(i==length)
        {
            ram_value = *(pData + 2*i);
            ram_value1 = 0xffffffff;
            flashdata = ((uint64_t)ram_value1 << 32) | ram_value;
            flash_address = USR_CONFIG_ADDR + i * 8;
        }
        
        // Clear all pending flags before each write operation
        fmc_flag_clear(FMC_FLAG_ENDF | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | 
                      FMC_FLAG_PGERR | FMC_FLAG_OPRERR | FMC_FLAG_PGSERR | 
                      FMC_FLAG_PGMERR);

        // Perform word programming
        fmc_state = fmc_doubleword_program(flash_address, flashdata);
        
        if (FMC_READY != fmc_state) {
            // Check if error is due to write protection
            if (fmc_state == FMC_WPERR) {
                fmc_flag_clear(FMC_FLAG_WPERR); // Clear write protection error
                fmc_lock();
                return -3; // Write protection error
            } else {
                fmc_lock();
                return -2; // Programming error
            }
        } else {
            savecnt++;
            testfmcdata = REG64(flash_address);
            // Optional: Verify t+he written data immediately
            if (REG64(flash_address) != flashdata) {
                fmc_lock();
                return -4; // Verification failed
            }
        }


    }
   // Step 5: Final verification of entire config block
    if (memcmp((void*)USR_CONFIG_ADDR, &UsrConfig, sizeof(tUsrConfig)) != 0) {
        fmc_lock();
        return -5; // Final verification failed
    }
    //Step 6: Lock flash controller
    fmc_lock();
    return 0;

}

void USR_CONFIG_set_default_cogging_map(void)
{
    if (pCoggingMap == NULL) {
        pCoggingMap = HEAP_malloc(sizeof(tCoggingMap));
    }

    for (int i = 0; i < COGGING_MAP_NUM; i++) {
        pCoggingMap->map[i] = 0;
    }
}

int USR_CONFIG_erease_cogging_map(void)
{
    uint32_t       addr;
    fmc_state_enum status;
    uint32_t page_number;

    fmc_unlock();

    // Erase
    for (addr = COGGING_MAP_ADDR; addr < (COGGING_MAP_ADDR + COGGING_MAP_MAX_SIZE); addr += PAGE_SIZE) {

        page_number = (addr - MAIN_FLASH_BASE_ADDRESS) / PAGE_SIZE;
        fmc_flag_clear(FMC_FLAG_ENDF | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | FMC_FLAG_PGERR);
        status = fmc_page_erase(FMC_BANK0,page_number);
        if (status != FMC_READY) {
            fmc_lock();
            return -1;
        }
    }

    fmc_lock();

    // Check
    for (addr = COGGING_MAP_ADDR; addr < (COGGING_MAP_ADDR + COGGING_MAP_MAX_SIZE); addr += 4) {
        if (0xFFFFFFFF != *((uint32_t *) addr)) {
            return -2;
        }
    }

    return 0;
}

int USR_CONFIG_read_cogging_map(void)
{
    int state = 0;

    if (pCoggingMap == NULL) {
        pCoggingMap = HEAP_malloc(sizeof(tCoggingMap));
    }

    memcpy(pCoggingMap, (uint8_t *) COGGING_MAP_ADDR, sizeof(tCoggingMap));

    uint32_t crc;
    crc = crc32((uint8_t *) pCoggingMap, sizeof(tCoggingMap) - 4);
    if (crc != pCoggingMap->crc) {
        state = -1;
    }

    return state;
}

uint32_t mapsavecnt=0;
uint16_t mapram_value=0,mapram_value1=0,mapram_value2=0,mapram_value3=0;
uint32_t a,b;
uint64_t mapflashdata = 0;
uint32_t mapflash_address=0;
uint64_t maptestfmcdata = 0;
uint32_t maplength = 0;
int USR_CONFIG_save_cogging_map(void)
{

    fmc_state_enum fmc_state = FMC_READY;
    
    // Step 1: Erase flash first
    if (USR_CONFIG_erease_cogging_map()) {
        return -1;
    }

    // Step 2: Unlock flash controller
    fmc_unlock();
    maplength =  sizeof(tCoggingMap)/8;
    // Step 3: Calculate CRC before programming
    pCoggingMap->crc = crc32((uint8_t *) pCoggingMap, sizeof(tCoggingMap) - 4);

    // Step 4: Program data word by word
    uint16_t *pData = (uint16_t *)pCoggingMap;
    

    for (int i = 0; i < sizeof(tCoggingMap) / 8 + 1; i++) {
        if(i<maplength)
        {
            mapram_value = *(pData + 4*i);
            mapram_value1 = *(pData + 4*i + 1);
            mapram_value2 = *(pData + 4*i + 2);
            mapram_value3 = *(pData + 4*i + 3);
            mapflashdata = ((uint64_t)mapram_value3 << 48) | ((uint64_t)mapram_value2 << 32) |  ((uint64_t)mapram_value1 << 16) | (uint64_t)mapram_value;
            mapflash_address = COGGING_MAP_ADDR + i * 8;
        }
        if(i==maplength)
        {
            uint32_t *pdata1 = (uint32_t*)pCoggingMap;
            a = *(pdata1+2500);
            b = 0xffffffff;
            mapflashdata = ((uint64_t)b << 32) | a;
            mapflash_address = COGGING_MAP_ADDR + i * 8;
        }
    
        
        // Clear all pending flags before each write operation
        fmc_flag_clear(FMC_FLAG_ENDF | FMC_FLAG_WPERR | FMC_FLAG_PGAERR | 
                      FMC_FLAG_PGERR | FMC_FLAG_OPRERR | FMC_FLAG_PGSERR | 
                      FMC_FLAG_PGMERR);

        // Perform word programming
        fmc_state = fmc_doubleword_program(mapflash_address, mapflashdata);
        
        if (FMC_READY != fmc_state) {
            // Check if error is due to write protection
            if (fmc_state == FMC_WPERR) {
                fmc_flag_clear(FMC_FLAG_WPERR); // Clear write protection error
                fmc_lock();
                return -3; // Write protection error
            } else {
                fmc_lock();
                return -2; // Programming error
            }
        } else {
            mapsavecnt++;
            maptestfmcdata = REG64(mapflash_address);
            // Optional: Verify t+he written data immediately
            if (REG64(mapflash_address) != mapflashdata) {
                fmc_lock();
                return -4; // Verification failed
            }
        }


    }
   // Step 5: Final verification of entire config block
    if (memcmp((void*)COGGING_MAP_ADDR, pCoggingMap, sizeof(tCoggingMap)) != 0) {
        fmc_lock();
        return -5; // Final verification failed
    }
    //Step 6: Lock flash controller
     fmc_lock();
     return 0;
}
