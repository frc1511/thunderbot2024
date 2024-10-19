// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <Controls/Controls.h>
#include <Drive/Drive.h>
#include <GamEpiece/Shamptake.h>
#include <Autonomous/Auto.h>
#include <GamEpiece/Arm.h>
#include <Hanger/Hang.h>
#include <BlinkyBlinky/BlinkyBlinky.h>
#include <Util/Limelight.h>

class Robot : public frc::TimedRobot {
public:
    void RobotInit() override;
    void RobotPeriodic() override;

    void AutonomousInit() override;
    void AutonomousPeriodic() override;

    void TeleopInit() override;
    void TeleopPeriodic() override;

    void DisabledInit() override;
    void DisabledPeriodic() override;

    void TestInit() override;
    void TestPeriodic() override;

private:
    void reset(Mechanism::MatchMode mode);
    bool debugMode = false;
    Mechanism::MatchMode lastMode = Mechanism::MatchMode::DISABLED;
    Drive drive;
    Arm arm;
    Shamptake shampTake{&arm};
    Hang hang;
    BlinkyBlinky blinky{&hang, &arm, &shampTake};
    Limelight limelight{&drive};
    
    //Controls controls {nullptr, &shampTake, &arm, &hang};
    Controls controls {&drive, &shampTake, &arm, &hang, &blinky, &limelight, &debugMode};
    Auto autoCode {&drive, &shampTake, &arm};
    std::vector<Mechanism*> allMechanisms {

        &arm, &hang, &drive, &shampTake, &controls, &autoCode, &blinky, &limelight

    };
};