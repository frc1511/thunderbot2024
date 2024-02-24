#pragma once

#include <Basic/IOMap.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkPIDController.h>
#include <rev/ControlType.h>
#include <frc/DigitalInput.h>
#include <Basic/Mechanism.h>
#include <Basic/IOMap.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#define SHAMPTANK_RIGHT_MOTOR_P 0.0002
#define SHAMPTANK_RIGHT_MOTOR_I 0.0
#define SHAMPTANK_RIGHT_MOTOR_D 0.0
#define SHAMPTANK_RIGHT_MOTOR_FEED_FOWARD 0.000170
#define SHAMPTANK_RIGHT_MOTOR_I_ZONE 0.0


#define SHAMPTANK_LEFT_MOTOR_P 0.0002
#define SHAMPTANK_LEFT_MOTOR_I 0.0
#define SHAMPTANK_LEFT_MOTOR_D 0.0
#define SHAMPTANK_LEFT_MOTOR_FEED_FOWARD 0.000170
#define SHAMPTANK_LEFT_MOTOR_I_ZONE 0.0
class Shamptake : public Mechanism{
  
  rev::CANSparkMax shooterMotorRight {CAN_SHOOTER_RIGHT, rev::CANSparkMax::MotorType::kBrushless}; //right side (top)
  rev::CANSparkMax shooterMotorLeft {CAN_SHOOTER_LEFT, rev::CANSparkMax::MotorType::kBrushless}; //left side (bottom)
  rev::CANSparkMax intakeMotor1 {CAN_SHOOTER_INTAKE, rev::CANSparkMax::MotorType::kBrushless};
  //rev::CANSparkMax intakeMotor2 {2, rev::CANSparkMax::MotorType::kBrushless};

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
    void runIntakeMotors();
    void autoIntake();
    void autoShoot();
    bool runIntake;
    bool runOuttake;

    enum IntakeSpeed {
      NORMAL,
      STOP,
      SLOW,
      FIRE,
      OUTTAKE
    };
    Shamptake::IntakeSpeed intakeSpeed = Shamptake::IntakeSpeed::NORMAL;
    frc::DigitalInput noteSensor{DIO_GAMEPIECE_RR_SENSOR};
    bool sensorDetected = false;
    bool trippedBefore = false;
    bool autoIntaking = false;
    bool autoShooting = false;

    frc::Timer shooterTimer;


  private:
    std::string intakeModeString();
    rev::SparkPIDController shooterMotorRightPIDController;
    rev::SparkPIDController shooterMotorLeftPIDController;
};



