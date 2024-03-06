#pragma once

#include <Basic/IOMap.h>
#include <Basic/Mechanism.h>
#include <Util/Preferences.h>

#include <frc/DigitalInput.h>
#include <rev/CANSparkMax.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/DutyCycleEncoder.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>

class Arm : public Mechanism {
public:
    Arm();
    ~Arm();

    void process() override;
    void sendFeedback() override;
    void doPersistentConfiguration() override;
    void resetToMode(MatchMode mode) override;

    bool isMoveDone();

    void setPower(double power);


    void stop();

    double getBoreNormalizedPosition();

    void setMotorBrake(bool armBrakeOn);

    enum Presets {
        BASE,
        LINE,
        AMP,
        SUBWOOFER,
        MEDIUM,
        TRAVEL,
        MAX_PRESETS
    };

    bool isNearPreset(Presets preset);

    void moveToAngle(units::angle::degree_t angle);

    void moveToPreset(Presets preset);

    bool isAtLowerLimit();

private:

    units::degree_t getRawBorePosition();

    units::degree_t getBoreDegrees();

    std::string getMotorModeString();

    frc::DutyCycleEncoder encoder{DIO_GAMEPIECE_BORE_ENCODER};
    rev::CANSparkMax armMotor {CAN_PIVOT_ARM, rev::CANSparkMax::MotorType::kBrushless};

    rev::SparkLimitSwitch forwardarmLimitSwitch = armMotor.GetForwardLimitSwitch(rev::SparkLimitSwitch::Type::kNormallyOpen);
    units::degree_t presetAngles [Presets::MAX_PRESETS] = {
        0_deg,
        30_deg,
        90_deg,
        10_deg,
        20.3_deg,
        5_deg
    };

    double targetAngleThreshold = 5;
    double presetAngleThreshold = 15;
    

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

    units::angle::degree_t targetAngle = 2_deg;

    frc::ProfiledPIDController<units::degrees> armPIDController{
        PREFERENCE_ARM.PID.Kp, PREFERENCE_ARM.PID.Ki, PREFERENCE_ARM.PID.Kd, 
        frc::TrapezoidProfile<units::degrees>::Constraints(PREFERENCE_ARM.PID.MaxVel, PREFERENCE_ARM.PID.MaxAccel)
    };
};