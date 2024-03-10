#include <Autonomous/Auto.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <Drive/Drive.h>
#include <GamEpiece/Shamptake.h>
#include <frc/DriverStation.h>

Auto::Auto(Drive* drive, Shamptake* shamptake, Arm* arm)
    : drive(drive), shamptake(shamptake), arm(arm) {

}
void Auto::resetToMode(MatchMode mode) {
    delayTimer.Reset();
    delayTimer.Start();
    autoTimer.Reset();
    autoTimer.Start();

    step = 0;

    if (mode == MatchMode::AUTO) {
        drive->calibrateIMU();
    }
}
void Auto::process() { //called during auto
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
        paths = &redPaths;
    }
    else {
        paths = &bluePaths;
    }

    // Autonomous delay.
    if (delayTimer.Get().value() <= frc::SmartDashboard::GetNumber("thunderdashboard_auto_start_delay", 0.0)) {
        return;
    }
    switch (mode) { //find what auto mode you are using and do it
        using enum AutoMode;
        case DO_NOTHING:
            doNothing();
            break;
        case TEST:
            test();
            break;
        case SPEAKER_1_GP:
            speaker1();
            break;
        case SPEAKER_2_GP:
            speaker2();
            break;
        case HAVOC:
            havoc();
            break;
        case BASIC_LOC_1:
            basic_loc_1();
            break;
        case BASIC_LOC_2:
            basic_loc_2();
            break;
        case BASIC_LOC_3:
            basic_loc_3();
            break;
        case SQUARE:
            squareTest();
            break;
    }
}

void Auto::test() { //test auto, leave, grab a note, and shoot
    if (step == 0 && arm->isMoveDone()) {
        frc::Pose2d initPose(paths->at(Path::SPEAKER_1).getInitialPose());
        drive->resetOdometry(frc::Pose2d(initPose.X(), initPose.Y(), initPose.Rotation().Degrees() - 90_deg));
        drive->runTrajectory(&paths->at(Path::SPEAKER_1), actions);
        step++;
    } else if (step == 1 && drive->isFinished()) {
        drive->runTrajectory(&paths->at(Path::SPEAKER_2_STAGE_2), actions);
        step++;
    }
}

void Auto::speaker1() {
    if (step == 0 && arm->isMoveDone()) {
        //Make shamptake function to make shooter go fast
        arm->moveToPreset(Arm::Presets::MEDIUM);
        frc::Pose2d initPose(paths->at(Path::SPEAKER_1).getInitialPose());
        drive->resetOdometry(frc::Pose2d(initPose.X(), initPose.Y(), initPose.Rotation().Degrees() - 90_deg));
        drive->runTrajectory(&paths->at(Path::SPEAKER_1), actions);
        step++;
    } else if (step == 1 && drive->isFinished() && arm->isMoveDone()) {
        shamptake->autoShoot();
        step++;
    } else if (step == 2 && shamptake->notShooting()) { //Make isShootingDone thing in shamptake
        step++;
    }
}

void Auto::speaker2() {
    speaker1();
    if (step == 3) {
        arm->moveToPreset(Arm::Presets::BASE);
        step++;
    } else if (step == 4 && arm->isMoveDone()) {
        shamptake->autoIntake();
        drive->runTrajectory(&paths->at(Path::SPEAKER_2_STAGE_2), actions);
        step++;
    } else if (step == 5 && drive->isFinished() && shamptake->autoIntakeFinished()) {
        arm->moveToPreset(Arm::Presets::MEDIUM);
        step++;
    } else if (step == 6 && arm->isMoveDone()) {
        shamptake->autoShoot();
        step++;
    } else if (step == 7 && shamptake->notShooting()) {
        step++;
    }
}

void Auto::havoc() {
    if (step == 0 && arm->isMoveDone()) {
        frc::Pose2d initPose(paths->at(Path::HAVOC).getInitialPose());
        drive->resetOdometry(frc::Pose2d(initPose.X(), initPose.Y(), initPose.Rotation().Degrees() - 90_deg));
        drive->runTrajectory(&paths->at(Path::HAVOC), actions);
        step++;
    }
}

void Auto::basic_loc_1() {
    if (step == 0 && arm->isMoveDone()) {
        frc::Pose2d initPose(paths->at(Path::BASIC_LOC_1).getInitialPose());
        drive->resetOdometry(frc::Pose2d(initPose.X(), initPose.Y(), initPose.Rotation().Degrees() - 90_deg));
        arm->moveToPreset(Arm::Presets::SUBWOOFER);
        step++;
    } else if (step == 1 && arm->isMoveDone()) {
        shamptake->autoShoot();
        step++;
    } else if (step == 2 && shamptake->notShooting()) {
        drive->runTrajectory(&paths->at(Path::BASIC_LOC_1), actions);
        arm->moveToPreset(Arm::Presets::BASE);
        shamptake->autoIntake();
        step++;
    } else if (step == 3 && shamptake->autoIntakeFinished() && drive->isFinished() && arm->isMoveDone()) {
        arm->moveToPreset(Arm::Presets::MEDIUM);
        step++;
    } else if (step == 4 && arm->isMoveDone()) {
        shamptake->autoShoot();
        step++;
    } else if (step == 5 && shamptake->notShooting()) {
        step++;
    }
}

void Auto::basic_loc_2() {
    if (step == 0 && arm->isMoveDone()) {
        preloaded = true;
        frc::Pose2d initPose(paths->at(Path::BASIC_LOC_2).getInitialPose());
        drive->resetOdometry(frc::Pose2d(initPose.X(), initPose.Y(), initPose.Rotation().Degrees() - 90_deg));
        arm->moveToPreset(Arm::Presets::BASE);
        step++;
    } else if (step == 1 && arm->isMoveDone()) {
        shamptake->autoShoot();
        preloaded = false;
        step++;
    } else if (step == 2 && shamptake->notShooting()) {
        drive->runTrajectory(&paths->at(Path::BASIC_LOC_2), actions);
        arm->moveToPreset(Arm::Presets::BASE);
        shamptake->autoIntake();
        step++;
    } else if (step == 3 && shamptake->autoIntakeFinished() && drive->isFinished() && arm->isMoveDone()) {
        arm->moveToPreset(Arm::Presets::LINE);
        step++;
    } else if (step == 4 && arm->isMoveDone()) {
        shamptake->autoShoot();
        step++;
    } else if (step == 5 && shamptake->notShooting()) {
        step++;
    }
}

void Auto::basic_loc_3() {
    if (step == 0 && arm->isMoveDone()) {
        frc::Pose2d initPose(paths->at(Path::BASIC_LOC_3).getInitialPose());
        drive->resetOdometry(frc::Pose2d(initPose.X(), initPose.Y(), initPose.Rotation().Degrees() - 90_deg));
        arm->moveToPreset(Arm::Presets::SUBWOOFER);
        step++;
    } else if (step == 1 && arm->isMoveDone()) {
        shamptake->autoShoot();
        step++;
    } else if (step == 2 && shamptake->notShooting()) {
        drive->runTrajectory(&paths->at(Path::BASIC_LOC_3), actions);
        arm->moveToPreset(Arm::Presets::BASE);
        shamptake->autoIntake();
        step++;
    } else if (step == 3 && shamptake->autoIntakeFinished() && drive->isFinished() && arm->isMoveDone()) {
        arm->moveToPreset(Arm::Presets::MEDIUM);
        step++;
    } else if (step == 4 && arm->isMoveDone()) {
        shamptake->autoShoot();
        step++;
    } else if (step == 5 && shamptake->notShooting()) {
        step++;
    }
}

    void Auto::squareTest() {
        if (step == 0 && arm->isMoveDone()) {
        frc::Pose2d initPose(paths->at(Path::SQUARE).getInitialPose());
        drive->resetOdometry(frc::Pose2d(initPose.X(), initPose.Y(), initPose.Rotation().Degrees() - 90_deg));
        drive->runTrajectory(&paths->at(Path::SQUARE), actions);
        step++;
    }
}

void Auto::doNothing() {
    // If it does nothing is it doing something or nothing? - trevor(2020)
        //it does something because it is doing nothing - ishan(2022)
        //I disagree - peter(2022)
        //I agree with peter -L Wrench
        //I still disagree with ishan - peter(2023)
        //it does something because this function exists and can be called as an action for the robot - ben d(2024)

    // Good function.
    // Very good function. - jeff downs
    // Very bad function. - jeff ups
    // I agree with jeff downs since he likes java - charlie(2024)

    // Well technically it's doing something - chris(2023)
}

void Auto::autoSelectorInit() {
    autoSelector.SetDefaultOption("Do Nothing", 0);
    autoSelector.AddOption("2 Note Loc 1", 4);
    autoSelector.AddOption("2 Note Loc 2", 5);
    autoSelector.AddOption("2 Note Loc 3", 6);
    autoSelector.AddOption("Square test" , 7);
}

void Auto::sendFeedback() {
    //int desiredAutoMode = static_cast<int>(frc::SmartDashboard::GetNumber("Auto_Mode", 0.0));
    int desiredAutoMode = autoSelector.GetSelected();
    if (desiredAutoMode < (int)autoModeNames.size() && desiredAutoMode >= 0) {
        mode = static_cast<AutoMode>(desiredAutoMode);
    }
    else {
        mode = AutoMode::DO_NOTHING;
    }

    frc::SmartDashboard::PutData("Auto Modes", &autoSelector);
    

    frc::SmartDashboard::PutNumber("Autonomous_Step", step);
    frc::SmartDashboard::PutBoolean("Autonomous_DriveFinished", drive->isFinished());
    frc::SmartDashboard::PutString("Autonomous_ModeName", autoModeNames.at(mode));

    std::string buffer = "";

    auto handleDashboardString = [&](AutoMode mode, const char* description) {
        int mode_index = static_cast<int>(mode);
        // Put mode index in buffer.
        buffer += fmt::format(",{}", mode_index);
        // Send description.
        frc::SmartDashboard::PutString(fmt::format("thunderdashboard_auto_{}", mode_index), description);
    };

    for (auto [mode, name] : autoModeNames) {
        handleDashboardString(mode, name);
    }

    frc::SmartDashboard::PutString("thunderdashboard_auto_list", buffer);
}
