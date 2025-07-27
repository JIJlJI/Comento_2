#ifndef PMIC_MP5475_H
#define PMIC_MP5475_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define PMIC_SLAVE_ADDRESS 0x60

typedef enum {
    REGISTER_UV_OV_STATUS = 0x07,
    REGISTER_OC_STATUS    = 0x08,
    REGISTER_TEMP_STATUS  = 0x09
} PmicRegisterAddress;

#pragma pack(push, 1)

typedef union {
    uint8_t raw;
    struct {
        uint8_t buckd_ov : 1; uint8_t buckc_ov : 1; uint8_t buckb_ov : 1;
        uint8_t bucka_ov : 1; uint8_t buckd_uv : 1; uint8_t buckc_uv : 1;
        uint8_t buckb_uv : 1; uint8_t bucka_uv : 1;
    } bits;
} RegisterUvOvStatus;

typedef union {
    uint8_t raw;
    struct {
        uint8_t buckd_oc_warning : 1; uint8_t buckc_oc_warning : 1;
        uint8_t buckb_oc_warning : 1; uint8_t bucka_oc_warning : 1;
        uint8_t buckd_oc : 1; uint8_t buckc_oc : 1;
        uint8_t buckb_oc : 1; uint8_t bucka_oc : 1;
    } bits;
} RegisterOcStatus;

typedef union {
    uint8_t raw;
    struct {
        uint8_t pmic_temp_shutdown : 1; uint8_t pmic_temp_warning : 1;
        uint8_t vdrv_ov : 1; uint8_t vbulk_ov : 1;
        uint8_t vr_fault : 1; uint8_t ldo1v1_fault : 1;
        uint8_t ldo1v8_fault : 1; uint8_t reserved : 1;
    } bits;
} RegisterSystemStatus;

#pragma pack(pop)

void pmic_request_fault_read_dma(void);
void pmic_i2c_dma_rx_callback_handler(void);

#endif /* PMIC_MP5475_H */




