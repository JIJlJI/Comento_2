#ifndef EEPROM_25LC256_H
#define EEPROM_25LC256_H

#include <stdint.h>
#include <stdbool.h>

#define DTC_MAX_COUNT 10
#define DTC_AREA_START_ADDRESS 100


bool eeprom_log_new_dtc(uint32_t dtc_code);
uint8_t eeprom_read_all_dtcs(uint32_t* dtc_buffer);
void eeprom_clear_all_dtcs(void);

#endif /* EEPROM_25LC256_H */
