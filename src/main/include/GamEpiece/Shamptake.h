#pragma once

#include <Basic/IOMap.h>
#include <Util/Preferences.h>
#include <GamEpiece/Arm.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkPIDController.h>
#include <rev/ControlType.h>
#include <frc/DigitalInput.h>
#include <Basic/Mechanism.h>
#include <Basic/IOMap.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Shamptake : public Mechanism{
  
  rev::CANSparkMax shooterMotorRight {CAN_SHOOTER_RIGHT, rev::CANSparkMax::MotorType::kBrushless}; //right side (top)
  rev::CANSparkMax shooterMotorLeft {CAN_SHOOTER_LEFT, rev::CANSparkMax::MotorType::kBrushless}; //left side (bottom)
  rev::CANSparkMax intakeMotor1 {CAN_SHOOTER_INTAKE, rev::CANSparkMax::MotorType::kBrushless};
  //rev::CANSparkMax intakeMotor2 {2, rev::CANSparkMax::MotorType::kBrushless};

  public:
    Shamptake(Arm* _arm/*, Auto* _auto*/);
    ~Shamptake();

    void process() override;
    void sendFeedback() override;
    void doPersistentConfiguration() override;
    void resetToMode(MatchMode mode) override;
    
    bool isNoteSensorTripped();

    void stopIntake();
    void shooter(double RPM);
    void stopShooter();
    void stop();

    void runMotors();

    bool atTargetRPM();
    bool notShooting();
    bool notIntaking();
    bool hasGamepiece();
    void overrideGamePieceState(bool state);

    void autoIntake();
    void autoShoot();

    bool runOuttake;

    enum IntakeSpeed {
        NORMAL_INTAKE,
        STOP_INTAKE,
        SLOW_INTAKE,
        FIRE_INTAKE,
        OUTTAKE_INTAKE,
        DEBOUNCE_INTAKE,
        MAX_INTAKE_SPEED
    };

    enum ShooterSpeed {
        STOP_SHOOTER,
        FIRE_SHOOTER,
        AMP_SHOOTER,
        AUTO_FIRE_SHOOTER,
        HALF_COURT_SHOOTER,
        MAX_SHOOTER_SPEED
    };

    Shamptake::IntakeSpeed intakeSpeed = Shamptake::IntakeSpeed::STOP_INTAKE;
    Shamptake::ShooterSpeed shooterSpeed = Shamptake::ShooterSpeed::STOP_SHOOTER;
    frc::DigitalInput noteSensor{DIO_GAMEPIECE_RR_SENSOR};
    bool sensorDetected = false;
    bool trippedBefore = false;
    bool autoIntaking = false;
    bool autoShooting = false;

    bool autoIntakeFinished();

    frc::Timer shooterTimer;

    void controlProcess(bool intakeButton, bool outtakeButton, bool fireButton, bool preheatButton, bool overrideGamePieceYes, bool overrideGamePieceNo, bool halfCourt);

    void autoSetToPreloadedState();

  private:
    void intake(double power);
    void debounceNote();
    std::string intakeModeString();
    bool isDebouncing = false;
    bool finishedDebouncing = false;
    rev::SparkPIDController shooterMotorRightPIDController;
    rev::SparkRelativeEncoder shooterMotorRightEncoder;
    rev::SparkPIDController shooterMotorLeftPIDController;
    rev::SparkRelativeEncoder shooterMotorLeftEncoder;
    Arm* arm;
    //Auto* autoCode; //Was working with this, but had to leave. Probably just didnt use it right, if i dont fix it, then can someone else try? Thx, 
    double targetShooterRPM = 0;
    bool isAuto = false;
    int step = 0;

    frc::Timer autoPreheatTimeout;
    frc::Timer autoIntakeTimeout;

    double presetIntakeSpeeds [IntakeSpeed::MAX_INTAKE_SPEED] = {
        0.9,
        0,
        0.4,
        0.8,
        -0.4,
        0.3
    };
    double presetShooterSpeeds [ShooterSpeed::MAX_SHOOTER_SPEED] = {
        0,
        2500,          // FIRE SHOOTER
        1000,          // AMP SHOOTER
        4500,          // AUTO_FIRE SHOOTER
        5000           // HALF_COURT SHOOTER
    };
    void configureShooterMotors();
};



