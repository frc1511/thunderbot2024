#pragma once


#include <ctre/Phoenix.h>
#include <frc/Joystick.h>
#include <rev/CANSparkMax.h>

class Drive {
    rev::CANSparkMax rightDriveMotor1{2, rev::CANSparkMax::MotorType::kBrushed}; 
    rev::CANSparkMax rightDriveMotor2{4, rev::CANSparkMax::MotorType::kBrushed};

    rev::CANSparkMax leftDriveMotor1{1, rev::CANSparkMax::MotorType::kBrushed};
    rev::CANSparkMax leftDriveMotor2{3, rev::CANSparkMax::MotorType::kBrushed};

    frc::Joystick driveController{1};

public:
    void move();
    double axisProccess(double rawAxis);

private:
    Drive* drive;
};