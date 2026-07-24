#!/usr/bin/env python
# encoding: utf-8

def build(bld):
    vehicle = bld.path.name
    bld.ap_stlib(
        name=vehicle + '_libs',
        ap_vehicle=vehicle,
        ap_libraries=bld.ap_common_vehicle_libraries() + [
            'AC_InputManager',
            'AP_InertialNav',
            'AP_Motors',
            'AC_AttitudeControl',
            'AP_AdvancedFailsafe',  # required by GCS_Common.cpp
            'AP_Avoidance',         # required by RC_Channel base class
            'AP_Winch',             # required by Lua bindings
            'AC_PrecLand',          # required by Lua bindings
            'AP_Follow',            # required by Lua bindings
        ],
    )

    bld.ap_program(
        program_name='ardublimp',
        program_groups=['bin', 'ardumotorblimp'],
        use=vehicle + '_libs',
        )
