// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <ctre/Phoenix.h>
#include <frc/Joystick.h>
#include <frc/DigitalInput.h>
#include <Shooter.h>
#include <Drive.h>

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
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  Shooter shooter;
  //Drive drive;
  frc::Joystick joystick1{0};
  frc::DigitalInput intakeNoteSensor{5};
  double shooterspeed = 0.5; // .2 is good for amp
  double outtakespeed = 0.8;
  double intakespeed = 0.6;
  int lastdpad = -1;
  
};
