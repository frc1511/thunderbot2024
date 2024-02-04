#pragma once

#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>
#include <Basic/Mechanism.h>
class Shamptake : public Mechanism{
  
  rev::CANSparkMax shooterMotorRight {9, rev::CANSparkMax::MotorType::kBrushless}; //right side (top)
  rev::CANSparkMax shooterMotorLeft {10, rev::CANSparkMax::MotorType::kBrushless}; //left side (bottom)
  rev::CANSparkMax intakeMotor1 {1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax intakeMotor2 {2, rev::CANSparkMax::MotorType::kBrushless};
  
  public:
    Shamptake();
    ~Shamptake();

    void process() override;
    void sendFeedback() override;
    void doPersistentConfiguration() override;
    void resetToMode(MatchMode mode) override;
    
    void intake(double Power);
    void stopIntake();
    void shooter(double Power);
    void stop();
    void shooterSwitch();//change mode between basic shooting and curved shooting
    void runIntakeMotors();
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



