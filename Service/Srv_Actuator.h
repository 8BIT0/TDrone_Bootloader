#ifndef __SRV_ACTUATOR_H
#define __SRV_ACTUATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "Bsp_GPIO.h"
#include "Bsp_Timer.h"
#include "Bsp_DMA.h"
#include "HW_Def.h"

#define DSHOT_CTL_MAX                   48
#define DSHOT_CTL_IDLE                  180
#define DSHOT_CTL_MIN                   1900

#define SERVO_CTL_MAX                   100
#define SERVO_CTL_IDLE                  200
#define SERVO_CTL_MIN                   300

#define ACTUATOR_STORAGE_SECTION_NAME   "Actuator_Para"
#define ACTUATOR_MOTO_COUNT             2
#define ACTUATOR_SERVO_COUNT            2

typedef enum
{
    Actuator_UProp = 0,
    Actuator_DProp,
    Actuator_ServoX,
    Actuator_ServoY,
    Actuator_PWM_SigSUM,
} SrvActuator_MotoTag_List;

typedef enum
{
    ESC_Type_DShot150 = 0,
    ESC_Type_DShot300,
    ESC_Type_DShot600,
    ESC_Type_DShot1200,
} SrvActuator_ESCType_List;

typedef enum
{
    Actuator_Ctl_Throttle = 0,
    Actuator_Ctl_GyrX,
    Actuator_Ctl_GyrY,
    Actuator_Ctl_GyrZ,
    Actuator_Ctl_Sum,
} SrvActuator_ChannelControl_List;

typedef struct
{
    uint8_t sig_id;

    int16_t ctl_val;
    int16_t max_val;
    int16_t min_val;
    int16_t idle_val;
    int16_t lock_val;
} SrvActuator_PWMOutObj_TypeDef;

typedef struct
{
    uint32_t time_stamp;

    uint16_t moto[ACTUATOR_MOTO_COUNT];
    uint16_t servo[ACTUATOR_SERVO_COUNT];
} SrvActuatorPipeData_TypeDef;

typedef struct
{
    bool (*init)(SrvActuator_ESCType_List ESC_type);
    bool (*lock)(void);
    void (*mix_control)(int16_t *p_val);
    bool (*set_spin_dir)(uint8_t index, uint8_t dir);
    bool (*moto_direct_drive)(uint8_t index, int16_t value);
    bool (*servo_direct_drive)(uint8_t index, uint16_t value);
} SrvActuator_TypeDef;

extern SrvActuator_TypeDef SrvActuator;

#ifdef __cplusplus
}
#endif

#endif