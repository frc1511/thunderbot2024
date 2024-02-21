#include <Autonomous/Auto.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <Drive/Drive.h>
#include <GamEpiece/Shamptake.h>

Auto::Auto(Drive* drive, Shamptake* shamptake)
    : drive(drive), shamptake(shamptake){

    }
void Auto::getAutonomousCommand() {
    /*frc::TrajectoryConfig trajectoryConfig{(units::meters_per_second_t)AUTO_MAX_SPEED,
                                           (units::meters_per_second_squared_t)AUTO_MAX_ANGLE_SPEED};
    trajectoryConfig.SetKinematics(drive->kinematics);

    frc::PIDController xController = frc::PIDController(1.5, 0, 0);
    frc::PIDController yController = frc::PIDController(1.5, 0, 0);
    

    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the +X direction
        frc::Pose2d{0_m, 0_m, 0_deg},
        // Pass through these two interior waypoints, making an 's' curve path
        {frc::Translation2d{1_m, 1_m}, frc::Translation2d{2_m, -1_m}},
        // End 3 meters straight ahead of where we started, facing forward
        frc::Pose2d{3_m, 0_m, 0_deg},
        // Pass the config
        trajectoryConfig);
    
    frc2::CommandPtr swerveControllerCommand = frc2::SwerveControllerCommand<4>(
        exampleTrajectory,
        [this]() {return drive->getEstimatedPose();},
        drive->kinematics,
        xController,
        yController,
        drive->manualThetaPIDController,
        [this](auto kinematics moduleStates) {drive->setModuleStates(kinematics moduleStates);},
        {}).ToPtr();   */
    
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
    if (step == 0) {
        drive->cmdDriveToPose(1_m, 0_m, 0_deg);
        step += 1;
    }
    if (step == 1 && drive->isTrajectoryFinished()) {
        shamptake->autoIntake();
        step += 1;
    }
    if (step == 2) {
        if (!shamptake->autoIntaking) {
            shamptake->autoShoot();
            step += 1;
        }
    }
    //Make the arm move!!!
    if (step == 3) {
        if (!shamptake->autoShooting) {
            step += 1;
        }
    }
    if (step == 4) {
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