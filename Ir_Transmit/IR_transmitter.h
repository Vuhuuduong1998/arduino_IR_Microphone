#ifndef _IR_TRANSMITTER_H_
#define _IR_TRANSMITTER_H_

#include "stdint.h"

#define IR_CODE_SECURITY_CH1          (0xE68E927D)   
#define IR_CODE_FULL_MAIN_LIGHT_CH1   (0xA659927D)
#define IR_CODE_INCREASE_CH1          (0xBA45927D)  
#define IR_CODE_DECREASE_CH1          (0xBB44927D)
#define IR_CODE_ON_OFF_CH1            (0xAB54927D)
#define IR_CODE_SLEEP_LIGHT_CH1       (0xBC43927D)
#define IR_CODE_TIMER_CH1             (0xAF50927D)

#define IR_CODE_ON_OLD_CH1            (0xBD42927D)
#define IR_CODE_OFF_OLD_CH1           (0xBE41927D)

#define IR_CODE_SECURITY_CH2          (0xE68D927D)   
#define IR_CODE_FULL_MAIN_LIGHT_CH2   (0x26D9927D)
#define IR_CODE_INCREASE_CH2          (0x3AC5927D)  
#define IR_CODE_DECREASE_CH2          (0x3BC4927D)
#define IR_CODE_ON_OFF_CH2            (0x2BD4927D)
#define IR_CODE_SLEEP_LIGHT_CH2       (0x3CC3927D)
#define IR_CODE_TIMER_CH2             (0x2FD0927D)

#define IR_CODE_ON_OLD_CH2            (0x3DC2927D)
#define IR_CODE_OFF_OLD_CH2           (0x3EC1927D)

void IR_send(uint32_t data);



#endif
