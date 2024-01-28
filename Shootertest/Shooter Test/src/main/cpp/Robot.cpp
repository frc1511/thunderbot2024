// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of

#include <Robot.h>
#include <Mechanisms.h>
#include <Shooter2.h>

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  mechanisms.Init(&shooter2);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  
}

void Robot::AutonomousPeriodic() {
 
}

void Robot::TeleopInit() {
}

void Robot::TeleopPeriodic() {
  double shooterFire = joystick1.GetRawAxis(3);
  bool runIntake = joystick1.GetRawButton(6);
  bool runOuttake = joystick1.GetRawButton(5);
  bool outtakeDownButton = joystick1.GetRawButtonPressed(7);
  bool outtakeUpButton = joystick1.GetRawButtonPressed(8);
  int dpad = joystick1.GetPOV(0);
  bool shooterSwitchButton = joystick1.GetRawButtonPressed(1); //this is the a button
  bool canRunIntake = true;
  // Can Run Intake?
  // Has Note + Shooter Button Pressed = Can Run Intake
  // Has Note + Shooter Button Not Pressed = CAN'T Run Intake
  // Doesn't Have Note + Shooter Button Pressed = Can Run Intake
  // Doesn't Have Note + Shooter Button Not Pressed =  Can Run Intake

  if (shooterSwitchButton) {
    mechanisms.ShooterSwitch();
  }

  //feedback for shooter switch
  printf("Shooter mode: %i \n", shooter2.shooterMode);

  if (shooter2.shooterMode == Shooter2::ShooterMode::DEFAULT) { 
    canRunIntake = intakeNoteSensor.Get();
    if (shooterFire >= .5) {
      shooter2.shooter2(shooterspeed);
      canRunIntake = true;
    } else {
      shooter2.stopShooter2();
    }
    if (runIntake && !intakeNoteSensor.Get()) {
      shooter2.stopIntake();
    }
    if (runIntake && canRunIntake && shooterFire >= .5) {
      shooter2.intake(1);
    } else if (runIntake && canRunIntake) {
      shooter2.intake(intakespeed);
    } else if (runOuttake) {
      shooter2.intake(-outtakespeed);
    } else {
      shooter2.stopIntake();
    }

    
    if (outtakeUpButton) {
      outtakespeed = outtakespeed + 0.1;
    } else if (outtakeDownButton) {
      outtakespeed = outtakespeed - 0.1;
    }


    if (lastdpad == -1){
      if (dpad == 0) {
        shooterspeed += 0.1;
      }
      else if (dpad == 180) {
        shooterspeed -= 0.1;
      }
      else if (dpad == 90) {
        intakespeed += 0.1;
      }
      else if (dpad == 270) {
        intakespeed -= 0.1;
      }
    }
    
    lastdpad = dpad;
  } else if (shooter2.shooterMode == Shooter2::ShooterMode::CURVED) {
    canRunIntake = intakeNoteSensor.Get();
    if (shooterFire >= .5) {
      shooter2.shooter2(shooterspeed);
      canRunIntake = true;
    } else {
      shooter2.stopShooter2();
    }
    if (runIntake && !intakeNoteSensor.Get()) {
      shooter2.stopIntake();
    }
    if (runIntake && canRunIntake && shooterFire >= .5) {
      shooter2.intake(1);
    } else if (runIntake && canRunIntake) {
      shooter2.intake(intakespeed);
    } else if (runOuttake) {
      shooter2.intake(-outtakespeed);
    } else {
      shooter2.stopIntake();
    }

    
    if (outtakeUpButton) {
      outtakespeed = outtakespeed + 0.1;
    } else if (outtakeDownButton) {
      outtakespeed = outtakespeed - 0.1;
    }


    if (lastdpad == -1){
      if (dpad == 0) {
        shooterspeed += 0.1;
      }
      else if (dpad == 180) {
        shooterspeed -= 0.1;
      }
      else if (dpad == 90) {
        intakespeed += 0.1;
      }
      else if (dpad == 270) {
        intakespeed -= 0.1;
      }
    }
    
    lastdpad = dpad;
  }

  //drive.move();
  //shooter.shoot();
  //double rightTrigger = joystick1.GetRawAxis(3);
  // double leftTrigger = joystick1.GetRawAxis(2);
  // bool rightBumper = joystick1.GetRawButton(6);
  // if (leftTrigger >= .5) { //50% deadzone for safety :)
  //   shooter.shoot(-.6); //Intake
  // } else if (joystick1.GetRawAxis(3) || joystick1.GetRawButton(6)) {
  //   if (joystick1.GetRawAxis(3)) { //pre heat (for better distance/height)
  //     shooter.shootTop(1);
  //   } 
  //   if (joystick1.GetRawButton(6)) { // shoot (hold this and the input above)
  //     shooter.shootBottom(1);
  //   }
  // } else {
  //   shooter.stop();
  // }



/*
// implementing smoother driving
  double driveYAxis = 0;
  driveYAxis = -1 * driveController.GetY();
  // if (driveYAxis > 0.2 || driveYAxis < -0.2){
  //   driveMotor.Set(ControlMode::PercentOutput, driveYAxis);
  // }
  // else{
  //   driveMotor.Set(ControlMode::PercentOutput, 0);
  // }

  // the better implentation of smootheer driving code
  if (driveYAxis > 0.2){
    driveYAxis -= 0.2;
    driveYAxis /= 0.2;
    driveMotor.Set(ControlMode::PercentOutput, driveYAxis);
  }
  else if (driveYAxis < -0.2){
    driveYAxis += 0.2;
    driveYAxis /= 0.8;
    driveMotor.Set(ControlMode::PercentOutput, driveYAxis);
  }
  else{
    driveMotor.Set(ControlMode::PercentOutput, 0);
  }
*/
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
