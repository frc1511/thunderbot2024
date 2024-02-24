#pragma once

#include <Basic/IOMap.h>
#include <Basic/Mechanism.h>
#include <frc/DigitalInput.h>
#include <rev/CANSparkMax.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/DutyCycleEncoder.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>

//might be differnet values
#define ARM_MOTOR_P 0.02
#define ARM_MOTOR_I 0.0
#define ARM_MOTOR_D 0.0
#define ARM_MAX_VEL 75_deg_per_s
#define ARM_MAX_ACCEL 75_deg_per_s_sq

class Arm : public Mechanism {
public:
    Arm();
    ~Arm();

    void  process() override;
    void sendFeedback() override;
    void doPersistentConfiguration() override;
    void resetToMode(MatchMode mode) override;

    bool isMoveDone();

    void setPower(double power);
    void moveToAngle(units::angle::degree_t angle);

    bool isAtAmp();

    void stop();

    void setMotorBrake(bool armBrakeOn);
private:
    bool init();

    units::degree_t getRawBorePosition();

    units::degree_t getBoreDegrees();

    std::string getMotorModeString();

    frc::DutyCycleEncoder encoder{DIO_GAMEPIECE_BORE_ENCODER};
    rev::CANSparkMax armMotor {CAN_PIVOT_ARM, rev::CANSparkMax::MotorType::kBrushless};

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

    units::angle::degree_t targetAngle = 10_deg;

    frc::ProfiledPIDController<units::degrees> armPIDController{
        ARM_MOTOR_P, ARM_MOTOR_I, ARM_MOTOR_D, 
        frc::TrapezoidProfile<units::degrees>::Constraints(ARM_MAX_VEL, ARM_MAX_ACCEL)
    };

};