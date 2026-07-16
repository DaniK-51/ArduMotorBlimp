#include "Blimp.h"

#if HAL_LOGGING_ENABLED

// Code to Write and Read packets from AP_Logger log memory
// Code to interact with the user to dump or erase logs

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

//Write a motor input packet
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

//Write a motor output packet
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

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    throttle_in;
    float    angle_boost;
    float    throttle_out;
    float    throttle_hover;
    float    desired_alt;
    float    inav_alt;
    int32_t  baro_alt;
    float    desired_rangefinder_alt;
    float    rangefinder_alt;
    float    terr_alt;
    int16_t  target_climb_rate;
    int16_t  climb_rate;
};

// Write an attitude packet
void Blimp::Log_Write_Attitude()
{
    //Attitude targets are all zero since Blimp doesn't have attitude control,
    //but the rest of the log message is useful.
    ahrs.Write_Attitude(Vector3f{0,0,0});
}

// Write an EKF and POS packet
void Blimp::Log_Write_EKF_POS()
{
    AP::ahrs().Log_Write();
}

struct PACKED log_Data_Int16t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    int16_t data_value;
};

// Write an int16_t data packet
UNUSED_FUNCTION
void Blimp::Log_Write_Data(LogDataID id, int16_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Int16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT16_MSG),
time_us     : AP_HAL::micros64(),
id          : (uint8_t)id,
data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt16t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    uint16_t data_value;
};

// Write an uint16_t data packet
UNUSED_FUNCTION
void Blimp::Log_Write_Data(LogDataID id, uint16_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_UInt16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT16_MSG),
time_us     : AP_HAL::micros64(),
id          : (uint8_t)id,
data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int32t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    int32_t data_value;
};

// Write an int32_t data packet
void Blimp::Log_Write_Data(LogDataID id, int32_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Int32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT32_MSG),
time_us  : AP_HAL::micros64(),
id          : (uint8_t)id,
data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt32t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    uint32_t data_value;
};

// Write a uint32_t data packet
void Blimp::Log_Write_Data(LogDataID id, uint32_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_UInt32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT32_MSG),
time_us     : AP_HAL::micros64(),
id          : (uint8_t)id,
data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Float {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    float data_value;
};

// Write a float data packet
UNUSED_FUNCTION
void Blimp::Log_Write_Data(LogDataID id, float value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Float pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_FLOAT_MSG),
time_us     : AP_HAL::micros64(),
id          : (uint8_t)id,
data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_ParameterTuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  parameter;     // parameter we are tuning, e.g. 39 is CH6_CIRCLE_RATE
    float    tuning_value;  // normalized value used inside tuning() function
    float    tuning_min;    // tuning minimum value
    float    tuning_max;    // tuning maximum value
};

void Blimp::Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, float tune_min, float tune_max)
{
    struct log_ParameterTuning pkt_tune = {
        LOG_PACKET_HEADER_INIT(LOG_PARAMTUNE_MSG),
time_us        : AP_HAL::micros64(),
parameter      : param,
tuning_value   : tuning_val,
tuning_min     : tune_min,
tuning_max     : tune_max
    };

    logger.WriteBlock(&pkt_tune, sizeof(pkt_tune));
}

// type and unit information can be found in
// libraries/AP_Logger/Logstructure.h; search for "log_Units" for
// units and "Format characters" for field type information
const struct LogStructure Blimp::log_structure[] = {
    LOG_COMMON_STRUCTURES,

    // @LoggerMessage: MOTORI
    // @Description: Motor input
    // @Field: TimeUS: Time since system startup
    // @Field: Y: Yaw
    // @Field: P: Pitch
    // @Field: R: Roll
    // @Field: X: X (forward/backward)

    {
        LOG_MOTORI_MSG, sizeof(log_MOTORI),
        "MOTORI",  "Qffff",     "TimeUS,Y,P,R,X", "s----", "F----"
    },

    // @LoggerMessage: MOTORO
    // @Description: Motor output
    // @Field: TimeUS: Time since system startup
    // @Field: M1: Motor 1 output
    // @Field: M2: Motor 2 output
    // @Field: M3: Motor 3 output
    // @Field: M4: Motor 4 output

    {
        LOG_MOTORO_MSG, sizeof(log_MOTORO),
        "MOTORO",  "Qffff",     "TimeUS,M1,M2,M3,M4", "s----", "F----"
    },

    // @LoggerMessage: PTUN
    // @Description: Parameter Tuning information
    // @URL: https://ardupilot.org/blimp/docs/tuning.html#in-flight-tuning
    // @Field: TimeUS: Time since system startup
    // @Field: Param: Parameter being tuned
    // @Field: TunVal: Normalized value used inside tuning() function
    // @Field: TunMin: Tuning minimum limit
    // @Field: TunMax: Tuning maximum limit

    {
        LOG_PARAMTUNE_MSG, sizeof(log_ParameterTuning),
        "PTUN", "QBfff",         "TimeUS,Param,TunVal,TunMin,TunMax", "s----", "F----"
    },

    // @LoggerMessage: CTUN
    // @Description: Control Tuning information
    // @Field: TimeUS: Time since system startup
    // @Field: ThI: throttle input
    // @Field: ABst: angle boost
    // @Field: ThO: throttle output
    // @Field: ThH: calculated hover throttle
    // @Field: DAlt: desired altitude
    // @Field: Alt: achieved altitude
    // @Field: BAlt: barometric altitude
    // @Field: DSAlt: desired rangefinder altitude
    // @Field: SAlt: achieved rangefinder altitude
    // @Field: TAlt: terrain altitude
    // @Field: DCRt: desired climb rate
    // @Field: CRt: climb rate

    // @LoggerMessage: D16
    // @Description: Generic 16-bit-signed-integer storage
    // @Field: TimeUS: Time since system startup
    // @Field: Id: Data type identifier
    // @Field: Value: Value

    // @LoggerMessage: DU16
    // @Description: Generic 16-bit-unsigned-integer storage
    // @Field: TimeUS: Time since system startup
    // @Field: Id: Data type identifier
    // @Field: Value: Value

    // @LoggerMessage: D32
    // @Description: Generic 32-bit-signed-integer storage
    // @Field: TimeUS: Time since system startup
    // @Field: Id: Data type identifier
    // @Field: Value: Value

    // @LoggerMessage: DFLT
    // @Description: Generic float storage
    // @Field: TimeUS: Time since system startup
    // @Field: Id: Data type identifier
    // @Field: Value: Value

    // @LoggerMessage: DU32
    // @Description: Generic 32-bit-unsigned-integer storage
    // @Field: TimeUS: Time since system startup
    // @Field: Id: Data type identifier
    // @Field: Value: Value

    {
        LOG_CONTROL_TUNING_MSG, sizeof(log_Control_Tuning),
        "CTUN", "Qffffffefffhh", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DSAlt,SAlt,TAlt,DCRt,CRt", "s----mmmmmmnn", "F----00B000BB"
    },

    {
        LOG_DATA_INT16_MSG, sizeof(log_Data_Int16t),
        "D16",   "QBh",         "TimeUS,Id,Value", "s--", "F--"
    },
    {
        LOG_DATA_UINT16_MSG, sizeof(log_Data_UInt16t),
        "DU16",  "QBH",         "TimeUS,Id,Value", "s--", "F--"
    },
    {
        LOG_DATA_INT32_MSG, sizeof(log_Data_Int32t),
        "D32",   "QBi",         "TimeUS,Id,Value", "s--", "F--"
    },
    {
        LOG_DATA_UINT32_MSG, sizeof(log_Data_UInt32t),
        "DU32",  "QBI",         "TimeUS,Id,Value", "s--", "F--"
    },
    {
        LOG_DATA_FLOAT_MSG, sizeof(log_Data_Float),
        "DFLT",  "QBf",         "TimeUS,Id,Value", "s--", "F--"
    },
};

uint8_t Blimp::get_num_log_structures() const
{
    return ARRAY_SIZE(log_structure);

}

void Blimp::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by AP_Logger
    logger.Write_MessageF("Frame: %s", get_frame_string());
    logger.Write_Mode((uint8_t)control_mode, control_mode_reason);
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_AP_Logger_Log_Startup_messages();
}

#endif // HAL_LOGGING_ENABLED
