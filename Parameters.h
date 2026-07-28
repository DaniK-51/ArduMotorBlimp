#pragma once

#define AP_PARAM_VEHICLE_NAME motorblimp

#include <AP_Param/AP_Param.h>

class Parameters {
public:
    static const uint16_t k_format_version = 1;

    enum {
        k_param_format_version = 0,
    };

    AP_Int16 format_version;
};

extern const AP_Param::Info var_info[];
