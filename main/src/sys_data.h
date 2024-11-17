#ifndef __SYS_DATA_H__

#include "esp_log.h"

typedef unsigned char UINT8;
typedef unsigned short UINT16;
typedef short SINT16;

#define COMBINE_TWO_BYTES(data_h, data_l) ((data_h << 8 | data_l))

#endif /* sys_data.h */