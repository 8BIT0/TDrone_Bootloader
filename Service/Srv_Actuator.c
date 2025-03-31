#include "Srv_Actuator.h"
#include "Srv_OsCommon.h"

#define SRV_ACTUATOR_MAX_THROTTLE_PERCENT 80
#define Actuator_Malloc(size) SrvOsCommon.malloc(size)
#define Actuator_Free(ptr) SrvOsCommon.free(ptr)

/* internal function */
static void SrvActuator_PipeData(void);

/* external function */
static bool SrvActuator_Init(SrvActuator_ESCType_List ESC_type);
static void SrvActuator_MotoControl(int16_t *p_val);
static bool SrvActuator_Lock(void);

/* external variable */
SrvActuator_TypeDef SrvActuator = {
    .init               = SrvActuator_Init,
    .lock               = SrvActuator_Lock,
    .mix_control        = SrvActuator_MotoControl,
    .set_spin_dir       = NULL,
    .moto_direct_drive  = NULL,
    .servo_direct_drive = NULL,
};

static bool SrvActuator_Init(SrvActuator_ESCType_List ESC_type)
{
    /* malloc dshot esc driver obj for using */
        
    /* default init */
    switch (ESC_type)
    {
        case ESC_Type_DShot150:
        case ESC_Type_DShot300:
        case ESC_Type_DShot600:
            /* set moto info */
            break;

        default: return false;
    }

    /* set servo object */

    /* check value remap relationship */
    /* we can read this info from storage module */
    SrvActuator_Lock();

    return true;
}

static bool SrvActuator_Lock(void)
{
}

static void SrvActuator_MotoControl(int16_t *p_val)
{
    // uint8_t i = 0;
    // uint8_t offset;

    // if ((p_val == NULL) || !SrvActuator_Obj.init)
    //     return;
}


