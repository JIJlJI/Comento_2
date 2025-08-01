#include "pmic_mp5475.h"
#include "main.h"
#include "cmsis_os.h"

extern I2C_HandleTypeDef hi2c1;
extern osMessageQueueId_t dtcProcessingQueueHandle;
extern volatile I2C_State_t g_i2c_state;
extern uint8_t g_i2c_rx_buffer[3];
extern volatile SystemState_t g_SystemState;

void pmic_request_fault_read_dma(void) {
    if (g_i2c_state != I2C_STATE_IDLE) {
        return;
    }
    g_i2c_state = I2C_STATE_READ_FAULTS_WAIT;
    uint16_t dev_address = PMIC_SLAVE_ADDRESS;
    HAL_I2C_Mem_Read_DMA(&hi2c1, dev_address, REGISTER_UV_OV_STATUS, I2C_MEMADD_SIZE_8BIT, g_i2c_rx_buffer, 3);
}

void pmic_i2c_dma_rx_callback_handler(void) {          // 인터럽트 발생하면 실행. fault여부를 확인.
    if (g_i2c_state == I2C_STATE_READ_FAULTS_WAIT)
    {
        RegisterUvOvStatus uv_ov_reg;
        uv_ov_reg.raw = g_i2c_rx_buffer[0];
        RegisterOcStatus oc_reg;
        oc_reg.raw = g_i2c_rx_buffer[1];
        RegisterSystemStatus temp_reg;
        temp_reg.raw = g_i2c_rx_buffer[2];

        DtcEvent_t event;
        event.command = DTC_EVENT_WRITE;


        //   SpiEepromTask을 깨움

        if (uv_ov_reg.bits.bucka_uv) {
            event.dtc_code = 0x0C1234;         // OBD-2 코드가 C1234로 가정 (C : Chassis. 브레이크 시스템)
            osMessageQueuePut(dtcProcessingQueueHandle, &event, 0U, 0U);
            g_SystemState = SYS_STATE_FAULT;
        }
        if (oc_reg.bits.bucka_oc) {
            event.dtc_code = 0x0C1235;
            osMessageQueuePut(dtcProcessingQueueHandle, &event, 0U, 0U);
            g_SystemState = SYS_STATE_FAULT;
        }
        if (temp_reg.bits.pmic_temp_shutdown) {
            event.dtc_code = 0x0C1236;
            osMessageQueuePut(dtcProcessingQueueHandle, &event, 0U, 0U);
            g_SystemState = SYS_STATE_FAULT;
        }

        g_i2c_state = I2C_STATE_IDLE;
    }
}

void pmic_set_vout(uint8_t register_addr, uint8_t vout_value) {
    HAL_I2C_Mem_Write(&hi2c1, PMIC_SLAVE_ADDRESS,
                      register_addr, I2C_MEMADD_SIZE_8BIT,
                      &vout_value, 1, HAL_MAX_DELAY);
}
