#include "eeprom_25lc256.h"
#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;
extern osSemaphoreId_t spiTxDoneSemaphoreHandle;

#define EEPROM_WREN_CMD  0x06
#define EEPROM_WRITE_CMD 0x02
#define EEPROM_READ_CMD  0x03

static void eeprom_cs_select(void) { HAL_GPIO_WritePin(EEPROM_CS_PORT, EEPROM_CS_PIN, GPIO_PIN_RESET); }
static void eeprom_cs_deselect(void) { HAL_GPIO_WritePin(EEPROM_CS_PORT, EEPROM_CS_PIN, GPIO_PIN_SET); }

static void eeprom_write_bytes(uint16_t address, uint8_t* data, uint16_t size) {
    uint8_t header[3];
    uint8_t wren_cmd = EEPROM_WREN_CMD;

    eeprom_cs_select();
    HAL_SPI_Transmit(&hspi1, &wren_cmd, 1, 100);
    eeprom_cs_deselect();
    osDelay(5);

    header[0] = EEPROM_WRITE_CMD;
    header[1] = (address >> 8) & 0xFF;
    header[2] = address & 0xFF;

    eeprom_cs_select();
    HAL_SPI_Transmit_DMA(&hspi1, header, 3);
    osSemaphoreAcquire(spiTxDoneSemaphoreHandle, 100);
    HAL_SPI_Transmit_DMA(&hspi1, data, size);
    osSemaphoreAcquire(spiTxDoneSemaphoreHandle, 100);
    eeprom_cs_deselect();
    osDelay(5);
}

static void eeprom_read_bytes(uint16_t address, uint8_t* data, uint16_t size) {
    uint8_t header[3];
    header[0] = EEPROM_READ_CMD;
    header[1] = (address >> 8) & 0xFF;
    header[2] = address & 0xFF;

    eeprom_cs_select();
    HAL_SPI_Transmit(&hspi1, header, 3, 100);
    HAL_SPI_Receive(&hspi1, data, size, 100);
    eeprom_cs_deselect();
}

bool eeprom_log_new_dtc(uint32_t dtc_code) {
    uint8_t buffer[4];
    uint16_t write_address = 0xFFFF;
    bool already_exists = false;

    for (int i = 0; i < DTC_MAX_COUNT; i++) {
        uint16_t current_addr = DTC_AREA_START_ADDRESS + (i * 4);
        eeprom_read_bytes(current_addr, buffer, 4);
        uint32_t existing_dtc = *((uint32_t*)buffer);
        if (existing_dtc == dtc_code) {
            already_exists = true;
            break;
        }
        if (write_address == 0xFFFF && existing_dtc == 0xFFFFFFFF) {
            write_address = current_addr;
        }
    }

    if (already_exists) {
        return true;
    }
    if (write_address != 0xFFFF) {
        buffer[0] = (dtc_code >> 24) & 0xFF; buffer[1] = (dtc_code >> 16) & 0xFF;
        buffer[2] = (dtc_code >> 8) & 0xFF; buffer[3] = dtc_code & 0xFF;
        eeprom_write_bytes(write_address, buffer, 4);
        return true;
    }

    printf("[EEPROM] Log area is full.\r\n");
    return false;
}

uint8_t eeprom_read_all_dtcs(uint32_t* dtc_buffer) {
    uint8_t count = 0;
    uint8_t read_bytes[4];
    for (int i = 0; i < DTC_MAX_COUNT; i++) {
        uint16_t current_addr = DTC_AREA_START_ADDRESS + (i * 4);
        eeprom_read_bytes(current_addr, read_bytes, 4);
        uint32_t current_dtc = *((uint32_t*)read_bytes);
        if (current_dtc != 0xFFFFFFFF) {
            dtc_buffer[count++] = current_dtc;
        }
    }
    return count;
}


void eeprom_clear_all_dtcs(void) {
    uint8_t empty_data[4];
    memset(empty_data, 0xFF, sizeof(empty_data));
    for (int i = 0; i < DTC_MAX_COUNT; i++) {
        uint16_t current_addr = DTC_AREA_START_ADDRESS + (i * 4);
        eeprom_write_bytes(current_addr, empty_data, 4);
    }
}
