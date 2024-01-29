#include <Auto/Auto.h>

Auto::Auto(Drive* drive)
    : drive(drive) {

    }

void Auto::doAuto() { //called during auto
    if (autoDone) { //don't do auto if you are done with auto
        doNothing();
        return;
    }

    switch (mode) { //find what auto mode you are using and do it
        case NONE:
            doNothing();
            break;
        case TEST:
            testAuto();
            break;
    }
}

void Auto::testAuto() { //test auto, leave
    printf("Auto Running\n");
    drive->setMode(Drive::DriveMode::VELOCITY);
    drive->moveDistance(5, 1_mps);
    //drive->execStopped();
    //autoDone = true;


  /*  if (step == 0) { //set the drive motors
        drive->drive();
        sleep(1000);
        drive->stop();
        step++;
    } else if (step >= 1) {
        autoDone = true; //auto is done after all steps
    }*/
}

void Auto::doNothing() {
    // If it does nothing is it doing something or nothing? - trevor(2020)
        //it does something because it is doing nothing - ishan(2022)
        //I disagree - peter(2022)
        //I agree with peter -L Wrench
        //I still disagree with ishan - peter(2023)

    // Good function.
    // Very good function. - jeff downs
    // Very bad function. - jeff ups
    // I agree with jeff downs since he likes java - charlie(2024)

    // Well technically it's doing something - chris(2023)
}