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
            'AP_Avoidance',
            'AP_LTM_Telem',
            'AP_Devo_Telem',
            'AP_KDECAN',
            'AP_AdvancedFailsafe',
            'AC_AttitudeControl',
            'AP_Motors',
        ],
    )

    bld.ap_program(
        program_name='ardublimp',
        program_groups=['bin', 'ardumotorblimp'],
        use=vehicle + '_libs',
        )
