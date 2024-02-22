#pragma once

#include <Basic/IOMap.h>
#include <Basic/Mechanism.h>
#include <frc/DigitalInput.h>
#include <rev/CANSparkMax.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/DutyCycle.h>
#define ARM_MINIMUM_ENCODER -42.309
#define ARM_MINIMUM_ENCODER_SLOW -35.000
#define ARM_MAXIMUM_ENCODER -3.000 // Approx.
#define ARM_MAXIMUM_ENCODER_SLOW -10.000

//might be differnet values
#define ARM_MOTOR_P 0.0002
#define ARM_MOTOR_I 0.0
#define ARM_MOTOR_D 0.0
#define ARM_MOTOR_FEED_FOWARD 0.000170
#define ARM_MOTOR_I_ZONE 0.0


class Arm : public Mechanism {
public:
    double ARM_SLOW_SPEED = 0.1;
    Arm();
    ~Arm();

    void process() override;
    void sendFeedback() override;
    void doPersistentConfiguration() override;
    void resetToMode(MatchMode mode) override;

    bool isOnLowerLimit();
    bool isAutoMovingArmDone();

    void setPower(double power);
    void autoMoveArm(units::angle::degree_t angle);
    void stopAutoMoveArm();

    void stop();
private:
    bool init();

    double getRawBorePosition();

    double getBoreDegrees();

    std::string getMotorModeString();

    frc::DigitalInput limitSwitch {DIO_ARM_LIMIT_SWITCH};
    frc::DigitalInput boreEncoder {DIO_GAMEPIECE_BORE_ENCODER};
    frc::DutyCycle encoder{boreEncoder};
    rev::CANSparkMax armMotor {CAN_PIVOT_ARM, rev::CANSparkMax::MotorType::kBrushless};

    bool armCanMove = true;

    //rev::SparkLimitSwitch forwardarmLimitSwitch = armMotor.GetForwardLimitSwitch(rev::SparkLimitSwitch::Type::kNormallyOpen);
    //rev::SparkLimitSwitch reversearmLimitSwitch = armMotor.GetReverseLimitSwitch(rev::SparkLimitSwitch::Type::kNormallyOpen);
    //rev::SparkRelativeEncoder armEncoder; // Encoder Inside of Motor

    /**
    * An alternate encoder object is constructed using the GetAlternateEncoder()
    * method on an existing CANSparkMax object. If using a REV Through Bore
    * Encoder, the type should be set to quadrature and the counts per
    * revolution set to 8192
    */
    //rev::SparkMaxAlternateEncoder boreEncoder;
    //rev::SparkRelativeEncoder boreEncoder; // Quadrature Resolution: 2048 Cycles per Revolution (8192 Counts per Revolution)

    bool backingOffMinimum = false;
    bool backingOffMaximum = false;

    units::angle::degree_t autoArmAngle;
    bool autoMovingArm = false;
    bool autoMovingArmDone = false;

    frc::ProfiledPIDController<units::degrees> armPIDController{ARM_MOTOR_P, ARM_MOTOR_I, ARM_MOTOR_D, frc::TrapezoidProfile<units::degrees>::Constraints()};

};