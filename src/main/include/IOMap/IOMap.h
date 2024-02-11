#pragma once
/*
██╗ ██████╗     ███╗   ███╗ █████╗ ██████╗ 
██║██╔═══██╗    ████╗ ████║██╔══██╗██╔══██╗
██║██║   ██║    ██╔████╔██║███████║██████╔╝
██║██║   ██║    ██║╚██╔╝██║██╔══██║██╔═══╝ 
██║╚██████╔╝    ██║ ╚═╝ ██║██║  ██║██║     
╚═╝ ╚═════╝     ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝     v2020
*/

// Uncomment to use test board
// #define TEST_BOARD

// DIO
#define DIO_INTAKE_BEAM 0
#define DIO_TRANSITION_BEAM 2
#define DIO_SHOOT_BEAM 3
#define DIO_DRIVE_ENC_LEFT_A 4
#define DIO_DRIVE_ENC_LEFT_B 5
#define DIO_DRIVE_ENC_RIGHT_A 6
#define DIO_DRIVE_ENC_RIGHT_B 7
#define DIO_INTAKE_RESET 8
#define DIO_HANG_RESET 9


// CAN
#define CAN_INTAKE_PIVOT 2
#define CAN_INTAKE_BEATERBARS 3
#define CAN_STORAGE_SHOOT_TRANSITION 5
#define CAN_SHOOT_PRIMER 11
#define CAN_SHOOT_LEFT 6
#define CAN_SHOOT_RIGHT 7
#define CAN_CONTROL_SPINNER 9 
#define CAN_HANG_WINCH 10
