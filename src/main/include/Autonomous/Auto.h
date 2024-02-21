#pragma once

#include <Drive/Drive.h>
#include <GamEpiece/Shamptake.h>
#include <frc/Timer.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/TrapezoidProfileCommand.h>

#define PHYSICAL_MAX_SPEED 5.0
#define PHYSICAL_MAX_ANGLE_SPEED 2 * 2 * 3.14159
#define AUTO_MAX_SPEED PHYSICAL_MAX_SPEED / 4
#define AUTO_MAX_ANGLE_SPEED PHYSICAL_MAX_ANGLE_SPEED / 10
#define AUTO_MAX_ACCELERATION 2
class Auto {

    public:
        Auto(Drive* drive, Shamptake* shamptake);

        enum AutoMode {
            NONE,
            TEST
        };

        bool isAutoDone() {  
             return autoDone; 
        };

        void doAuto();

        void getAutonomousCommand();

    private:
        Drive* drive;
        Shamptake* shamptake;

        Auto::AutoMode mode = AutoMode::TEST;

        frc::Timer delayTimer {};
        frc::Timer autoTimer {};

        void doNothing();
        void testAuto();

        int step = 0;
        bool autoDone;
};

// class DriveDistanceProfiled
//     : public frc2::CommandHelper<frc2::TrapezoidProfileCommand<units::meters>,
//                                  DriveDistanceProfiled> {
//     public:
//     DriveDistanceProfiled(units::meter_t distance, Drive* drive);
// };