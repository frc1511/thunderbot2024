#include <Autonomous/Auto.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <Drive/Drive.h>
#include <GamEpiece/Shamptake.h>

Auto::Auto(Drive* drive, Shamptake* shamptake, Arm* arm)
    : drive(drive), shamptake(shamptake), arm(arm) {

}
void Auto::reset() {
    step = 0;
    drive->resetOdometry({0_m, 0_m, 0_deg});
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
        case SPEAKER1:
            speaker1Auto();
            break;
    }
}

void Auto::testAuto() { //test auto, leave, grab a note, and shoot
    if (step == 0) {
        drive->cmdDriveToPose(0_m, 3_m, 0_deg);
        step++;
    }
    if (step == 1 && drive->isTrajectoryFinished()) {
        autoDone = true;
    }
       // drive->setMode(Drive::DriveMode::VELOCITY);
   // drive->moveDistance(5, 1_mps);
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

void Auto::speaker1Auto() {
    if (step == 0) {
        drive->cmdDriveToPose(0_m, 0.94_m, -45_deg);
        arm->moveToAngle(20.3_deg);
        step++;
    }
    if (step == 1 && drive->isTrajectoryFinished() && arm->isMoveDone()) {
        shamptake->autoShoot();
        step++;
    }
    if (step == 2 && !shamptake->autoShooting) {
        autoDone = true;
    }
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

// DriveDistanceProfiled::DriveDistanceProfiled(units::meter_t distance,
//                                              Drive* drive)
//     : CommandHelper{
//           frc::TrapezoidProfile<units::meters>{
//               // Limit the max acceleration and velocity
//               {AUTO_MAX_SPEED, AUTO_MAX_ACCELERATION}},
//           // Pipe the profile state to the drive
//           [drive](auto setpointState) {
//             drive->setModuleStates(setpointState, setpointState);
//           },
//           // End at desired position in meters; implicitly starts at 0
//           [distance] {
//             return frc::TrapezoidProfile<units::meters>::State{distance, 0_mps};
//           },
//           [] { return frc::TrapezoidProfile<units::meters>::State{}; },
//           // Require the drive
//           {drive}} {
//   // Reset drive encoders since we're starting at 0
//   drive->resetOdometry();
// }