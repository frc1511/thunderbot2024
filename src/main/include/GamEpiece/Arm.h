#pragma once

#include <Basic/Mechanism.h>
#include <frc/DigitalInput.h>
#include <rev/CANSparkMax.h>

#define ARM_MINIMUM_ENCODER -42.309
#define ARM_MINIMUM_ENCODER_SLOW -35.000
#define ARM_MAXIMUM_ENCODER -3.000 // Approx.
#define ARM_MAXIMUM_ENCODER_SLOW -10.000

#define ARM_SLOW_SPEED 0.1

class Arm : public Mechanism {
public:
    Arm();
    ~Arm();

    void process() override;
    void sendFeedback() override;
    void doPersistentConfiguration() override;
    void resetToMode(MatchMode mode) override;

    bool isOnLowerLimit();

    void setPower(double power);

    void stop();
private:
    bool init();

    double getRawMotorPosition();

    double getRawMotorRotationPosition();

    frc::DigitalInput limitSwitch {6};
    rev::CANSparkMax armMotor {18, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkRelativeEncoder armEncoder;

    bool backingOffMinimum = false;
    bool backingOffMaximum = false;
};