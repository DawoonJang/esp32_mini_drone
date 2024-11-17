#ifndef __SYS_DATA_H__

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef unsigned char UINT8;
typedef unsigned short UINT16;
typedef short SINT16;

#define COMBINE_TWO_BYTES(data_h, data_l) ((data_h << 8 | data_l))
#define DELAY_MOTOR 100

#endif /* sys_data.h */
