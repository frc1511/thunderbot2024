#pragma once

#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>

class Shamptake {
  
  rev::CANSparkMax shooterMotor1 {1, rev::CANSparkMax::MotorType::kBrushless}; //right side (top)
  rev::CANSparkMax shooterMotor2 {2, rev::CANSparkMax::MotorType::kBrushless}; //left side (bottom)
  rev::CANSparkMax intakeMotor1 {3, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax intakeMotor2 {4, rev::CANSparkMax::MotorType::kBrushless};
  
  public:
    void intake(double Power);
    void stopIntake();
    void shooter(double Power);
    void stop();
    void shooterSwitch();//change mode between basic shooting and curved shooting
    void shamptakeProcess();
    void runIntakeMotors();
    double motor1power;
    double motor2power;
    bool runIntake;
    bool runOuttake;

    enum ShooterMode {
      DEFAULT,
      CURVED
    };

    enum IntakeSpeed {
      NORMAL,
      STOP,
      SLOW,
      FIRE,
      OUTTAKE
    };
    Shamptake::IntakeSpeed intakeSpeed = Shamptake::IntakeSpeed::NORMAL;
    frc::DigitalInput noteSensor{5};
    bool sensorDetected = false;
    bool trippedBefore = false;

    Shamptake::ShooterMode shooterMode = Shamptake::ShooterMode::DEFAULT;
};



