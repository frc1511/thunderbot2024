#pragma once


/*
    ██╗ ██████╗     ███╗   ███╗ █████╗ ██████╗ 
    ██║██╔═══██╗    ████╗ ████║██╔══██╗██╔══██╗
    ██║██║   ██║    ██╔████╔██║███████║██████╔╝
    ██║██║   ██║    ██║╚██╔╝██║██╔══██║██╔═══╝ 
    ██║╚██████╔╝    ██║ ╚═╝ ██║██║  ██║██║     
    ╚═╝ ╚═════╝     ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝     
*/


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

#define CAN_INTAKE_MOTOR 15
#define CAN_SHOOTER_LEFT_MOTOR 16
#define CAN_SHOOTER_RIGHT_MOTOR 17
#define CAN_ARM_PIVOT_MOTOR 18
#define CAN_ARM_BRAKE_MOTOR 19

#define CAN_HANG_ARM_RIGHT 20
#define CAN_HANG_ARM_LEFT 21


//number for this isnt actually on IOMap yet so change as needed based on what makes sense once we've finalized sensors more
#define PWM_BLINKY_BLINKY 0



#define CAN_PIGEON 14