#pragma once
// Last Update: 2/17/2024 - 2:54:09 PM
// https://wiki.penfieldrobotics.com/wiki/index.php?title=2024:Robot_IO_Map



/*
    ██╗ ██████╗     ███╗   ███╗ █████╗ ██████╗ 
    ██║██╔═══██╗    ████╗ ████║██╔══██╗██╔══██╗
    ██║██║   ██║    ██╔████╔██║███████║██████╔╝
    ██║██║   ██║    ██║╚██╔╝██║██╔══██║██╔═══╝ 
    ██║╚██████╔╝    ██║ ╚═╝ ██║██║  ██║██║     
    ╚═╝ ╚═════╝     ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝     
*/
#define CAN_DO_NOT_USE 1

#define CAN_SWERVE_DRIVE_FR 2
#define CAN_SWERVE_DRIVE_FL 3
#define CAN_SWERVE_DRIVE_BR 4
#define CAN_SWERVE_DRIVE_BL 5

#define CAN_SWERVE_ROTATION_FR 6
#define CAN_SWERVE_ROTATION_FL 7
#define CAN_SWERVE_ROTATION_BR 8
#define CAN_SWERVE_ROTATION_BL 9

#define CAN_SWERVE_CANCODER_FR 10
#define CAN_SWERVE_CANCODER_FL 11
#define CAN_SWERVE_CANCODER_BR 12
#define CAN_SWERVE_CANCODER_BL 13

#define CAN_PIGEON 14
#define CAN_SHOOTER_INTAKE 15
#define CAN_SHOOTER_LEFT 16
#define CAN_SHOOTER_RIGHT 17

#define CAN_PIVOT_ARM 18
#define CAN_PIVOT_ARM_BRAKE 19

#define CAN_HANG_ARM_RIGHT 20
#define CAN_HANG_ARM_LEFT 21

#define PWM_BLINKY_BLINKY 0
#define PWM_SLOT_1 1
#define PWM_SLOT_2 2
#define PWM_SLOT_3 3
#define PWM_SLOT_4 4
#define PWM_SLOT_5 5
#define PWM_SLOT_6 6
#define PWM_SLOT_7 7
#define PWM_SLOT_8 8
#define PWM_SLOT_9 9

#define DIO_GAMEPIECE_RR_SENSOR 0
#define DIO_GAMEPIECE_BORE_ENCODER 1
#define DIO_HANG_LIMIT_SWITCH_RIGHT 2
#define DIO_HANG_LIMIT_SWITCH_LEFT 3
#define DIO_HANG_RR_SENSOR_LEFT 4
#define DIO_HANG_RR_SENSOR_RIGHT 5
#define DIO_ARM_LIMIT_SWITCH 6
#define DIO_SLOT_7 7
#define DIO_SLOT_8 8
#define DIO_SLOT_9 9

#define RELAY_HANG_GEAR_LOCK 0
#define RELAY_SLOT_1 1
#define RELAY_SLOT_2 2
#define RELAY_SLOT_3 3

#define ANALOG_SLOT_0 0
#define ANALOG_SLOT_1 1
#define ANALOG_SLOT_2 2
#define ANALOG_SLOT_3 3