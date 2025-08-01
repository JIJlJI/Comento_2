#include "eeprom_25lc256.h"
#include "cmsis_os.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;
extern osSemaphoreId_t spiTxDoneSemaphoreHandle;

// 전역 변수
DTC_Table_t DTC_Table = { 0x1234, "Brake UV Fault", 0 };

// CS 핀 제어 (CS핀이 low일때만 명령을 받음)
static inline void eeprom_cs_select(void) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); }
static inline void eeprom_cs_deselect(void) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); }

// Status Register 읽기 (WIP 확인용. 반환된 값의 Bit0을 확인하여 쓰기 중인지 확인할 수 있음)
static uint8_t eeprom_read_status(void) {
    uint8_t cmd = EEPROM_CMD_RDSR;
    uint8_t status = 0;

    eeprom_cs_select();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &status, 1, HAL_MAX_DELAY);
    eeprom_cs_deselect();

    return status;
}

// WIP(Bit0)이 0이 될 때까지 대기
static void eeprom_wait_until_ready(void) {
    while (eeprom_read_status() & 0x01) {   // 쓰기 중인지 계속 확인. status가 0이 되어야 EEPROM이 준비 완료. 다음작업을 수행함
        osDelay(1);
    }
}

// WREN 실행. EEPROM은 쓰기 가능 상태(WEL=1) 로 만들어야만 데이터를 쓸 수 있음->쓰기 전 WREN 명령을 보내줌
static void eeprom_write_enable(void) {
    uint8_t cmd = EEPROM_CMD_RDSR;
    eeprom_cs_select();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    eeprom_cs_deselect();
}

// 바이트 쓰기 (DMA 사용)
static void eeprom_write_bytes(uint16_t address, uint8_t* data, uint16_t size) {
    uint8_t header[3];
    uint8_t wren_cmd = EEPROM_WREN_CMD;

    // WREN 명령어 전송 (쓰기 활성화)
    eeprom_cs_select();
    HAL_SPI_Transmit(&hspi1, &wren_cmd, 1, 100);
    eeprom_cs_deselect();
    osDelay(5);

    // WRITE 명령어, 쓰기 시작 주소 설정
    header[0] = EEPROM_WRITE_CMD;
    header[1] = (address >> 8) & 0xFF;  // 상위 8비트만 남기고 나머지 비트 제거
    header[2] = address & 0xFF;         // 하위 8비트만 남김

    // DMA로 명령어, 주소 전송
    eeprom_cs_select();
    HAL_SPI_Transmit_DMA(&hspi1, header, 3);
    osSemaphoreAcquire(spiTxDoneSemaphoreHandle, 100);  // DMA 완료 기다림

    // DMA로 실제 데이터 전송
    HAL_SPI_Transmit_DMA(&hspi1, data, size);
    osSemaphoreAcquire(spiTxDoneSemaphoreHandle, 100);  // DMA 완료 기다림
    eeprom_cs_deselect();

    // EEPROM이 내부적으로 데이터를 다 쓸 때까지 대기
    osDelay(5);
}

// 바이트 읽기 (Polling) (읽기의 경우 데이터가 작아 polling이 간단)
static void eeprom_read_bytes(uint16_t address, uint8_t *data, uint16_t size) {
    uint8_t header[3] = {
    	EEPROM_CMD_RDSR,
        (uint8_t)(address >> 8),
        (uint8_t)(address & 0xFF)
    };

    eeprom_cs_select();
    HAL_SPI_Transmit(&hspi1, header, 3, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, data, size, HAL_MAX_DELAY);
    eeprom_cs_deselect();
}



// DTC_Table을 EEPROM에 기록
void EEPROM_WriteDTC(void) {
    eeprom_write_bytes(EEPROM_DTC_ADDR, (uint8_t*)&DTC_Table, sizeof(DTC_Table));
}

// EEPROM에서 DTC_Table 읽기
void EEPROM_ReadDTC(void) {
    eeprom_read_bytes(EEPROM_DTC_ADDR, (uint8_t*)&DTC_Table, sizeof(DTC_Table));
}

// 새로운 DTC 추가 (중복 방지)
bool EEPROM_LogNewDTC(uint32_t dtc_code) {
    uint8_t buffer[4];               //DTC 코드 저장공간
    uint16_t write_address = 0xFFFF; // 새 코드 저장 주소
    bool already_exists = false;

    for (int i = 0; i < 32; i++) { // DTC_MAX_COUNT = 32 가정
        uint16_t addr = EEPROM_DTC_ADDR + sizeof(DTC_Table) + (i * 4);  // eeprom에서의 주소 계산. 기본 시작 주소 + dtc 테이블 크기 + 각 코드*4(각코드는 4byte)
        eeprom_read_bytes(addr, buffer, 4);    // addr 주소에서 4바이트 읽어서 buffer에 저장

        uint32_t existing_dtc =        //읽은 데이터를 32비트로 조립
            ((uint32_t)buffer[0] << 24) |
            ((uint32_t)buffer[1] << 16) |
            ((uint32_t)buffer[2] << 8)  |
            (uint32_t)buffer[3];

        if (existing_dtc == dtc_code) {          // 중복여부 호가인
            already_exists = true;
            break;
        }
        if (write_address == 0xFFFF && existing_dtc == 0xFFFFFFFF) {    // 빈공간찾기. 읽은 값이 모두 1이면 빈공간, 주소가 모두 1이면 아직 주소를 찾지 못함.
            write_address = addr;
        }
    }

    if (already_exists) return true;
    if (write_address == 0xFFFF) {
        printf("[EEPROM] Log area full\r\n");
        return false;
    }

    buffer[0] = (dtc_code >> 24) & 0xFF;    // dtc를 4바이트로 분해해서 버퍼에 넣음
    buffer[1] = (dtc_code >> 16) & 0xFF;
    buffer[2] = (dtc_code >> 8) & 0xFF;
    buffer[3] = dtc_code & 0xFF;

    eeprom_write_bytes(write_address, buffer, 4);
    return true;
}

// 모든 DTC 읽기
uint8_t EEPROM_ReadAllDTCs(uint32_t *dtc_buffer) {
    uint8_t count = 0;
    uint8_t buf[4];

    for (int i = 0; i < 32; i++) {
        uint16_t addr = EEPROM_DTC_ADDR + sizeof(DTC_Table) + (i * 4);
        eeprom_read_bytes(addr, buf, 4);

        uint32_t dtc =
            ((uint32_t)buf[0] << 24) |
            ((uint32_t)buf[1] << 16) |
            ((uint32_t)buf[2] << 8)  |
            (uint32_t)buf[3];

        if (dtc != 0xFFFFFFFF) {
            dtc_buffer[count++] = dtc;
        }
    }
    return count;
}

// 모든 DTC 초기화
void EEPROM_ClearAllDTCs(void) {
    uint8_t empty[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
    for (int i = 0; i < 32; i++) {
        uint16_t addr = EEPROM_DTC_ADDR + sizeof(DTC_Table) + (i * 4);
        eeprom_write_bytes(addr, empty, 4);
    }
}
