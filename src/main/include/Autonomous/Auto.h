#pragma once

#include <Drive/Drive.h>
#include <frc/Timer.h>

class Auto {

    public:
        Auto(Drive* drive);

        enum AutoMode {
            NONE,
            TEST,
            SIX_NOTE
        };

        bool isAutoDone() {  
             return autoDone; 
        };

        void doAuto();
    
    private:
        Drive* drive;

        Auto::AutoMode mode = AutoMode::TEST;

        frc::Timer delayTimer {};
        frc::Timer autoTimer {};

        void doNothing();
        void testAuto();

        int step = 0;
        bool autoDone;
};