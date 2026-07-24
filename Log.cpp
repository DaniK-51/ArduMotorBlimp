#include "Blimp.h"

#if HAL_LOGGING_ENABLED

struct PACKED log_MOTORI {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float Yaw;
    float Pitch;
    float Roll;
    float X;
};

struct PACKED log_MOTORO {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float M1;
    float M2;
    float M3;
    float M4;
};

void Blimp::Write_MOTORI(float yaw, float pitch, float roll, float x)
{
    const struct log_MOTORI pkt {
        LOG_PACKET_HEADER_INIT(LOG_MOTORI_MSG),
        time_us       : AP_HAL::micros64(),
        Yaw           : yaw,
        Pitch         : pitch,
        Roll          : roll,
        X             : x
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Blimp::Write_MOTORO(float *outputs)
{
    const struct log_MOTORO pkt {
        LOG_PACKET_HEADER_INIT(LOG_MOTORO_MSG),
        time_us       : AP_HAL::micros64(),
        M1            : outputs[0],
        M2            : outputs[1],
        M3            : outputs[2],
        M4            : outputs[3],
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

const struct LogStructure Blimp::log_structure[] = {
    LOG_COMMON_STRUCTURES,

    {
        LOG_MOTORI_MSG, sizeof(log_MOTORI),
        "MOTORI",  "Qffff",     "TimeUS,Y,P,R,X", "s----", "F----"
    },

    {
        LOG_MOTORO_MSG, sizeof(log_MOTORO),
        "MOTORO",  "Qffff",     "TimeUS,M1,M2,M3,M4", "s----", "F----"
    },
};

uint8_t Blimp::get_num_log_structures() const
{
    return ARRAY_SIZE(log_structure);
}

void Blimp::Log_Write_Vehicle_Startup_Messages()
{
    logger.Write_MessageF("Frame: %s", get_frame_string());
    logger.Write_Mode((uint8_t)control_mode, control_mode_reason);
}

#endif // HAL_LOGGING_ENABLED
