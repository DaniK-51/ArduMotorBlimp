#!/usr/bin/env python3

def build(bld):
    vehicle = bld.path.name
    bld.ap_stlib(
        name=vehicle + '_libs',
        ap_vehicle=vehicle,
        ap_libraries=bld.ap_common_vehicle_libraries() + [
            'AP_LTM_Telem',
            'AP_Devo_Telem',
        ],
    )
    bld.ap_program(
        program_name='ardumotorblimp',
        program_groups=['bin', 'motorblimp'],
        use=vehicle + '_libs',
    )
