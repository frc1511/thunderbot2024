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
    }
}

void Auto::test() { //test auto, leave, grab a note, and shoot
    if (step == 0) {
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
    if (step == 0) {
        //Make shamptake function to make shooter go fast
        arm->moveToPreset(Arm::Presets::LINE);
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
    } else if (step == 4) {
        shamptake->autoIntake();
        drive->runTrajectory(&paths->at(Path::SPEAKER_2_STAGE_2), actions);
        step++;
    } else if (step == 5 && drive->isFinished() && shamptake->hasGamepiece()) {
        arm->moveToPreset(Arm::Presets::LINE);
        step++;
    } else if (step == 6 && arm->isMoveDone()) {
        shamptake->autoShoot();
        step++;
    } else if (step == 7 && shamptake->notShooting()) {
        step++;
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

void Auto::sendFeedback() {
    int desiredAutoMode = static_cast<int>(frc::SmartDashboard::GetNumber("Auto_Mode", 0.0));
    if (desiredAutoMode < (int)autoModeNames.size() && desiredAutoMode >= 0) {
        mode = static_cast<AutoMode>(desiredAutoMode);
    }
    else {
        mode = AutoMode::DO_NOTHING;
    }


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
