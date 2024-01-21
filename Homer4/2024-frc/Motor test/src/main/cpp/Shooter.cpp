#pragma once

#include <Shooter.h>

void Shooter::intake(double Power) {
    feederMotor.Set(Power);
}
void Shooter::stopIntake() {
    intake(0);
}
void Shooter::shooter(double Power) {
    intakeMotor1.Set(Power);
    intakeMotor2.Set(Power);
}
void Shooter::stopShooter() {
    shooter(0);
}
void Shooter::stop() {
    intake(0);
    shooter(0);
}

Shooter::Shooter() {
    intakeMotor1.SetInverted(false);
    intakeMotor2.SetInverted(false);
    feederMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
}


// void Shooter::shoot(double Power) {
//         intakeMotor1.Set(ControlMode::PercentOutput, Power);
//         shooterMotor2.Set(ControlMode::PercentOutput, Power);
// }

// void Shooter::shootTop(double Power) {
//         shooterMotor2.Set(ControlMode::PercentOutput, Power);
// }

// void Shooter::shootBottom(double Power) {
//         shooterMotor1.Set(ControlMode::PercentOutput, Power);
// }

// void Shooter::stop() {
//     shoot(0);
// }

// void Shooter::food() {
//     static bool processing = false;
//     if (!processing) {
//         processing = true;
//         shooterMotor2.Set(ControlMode::PercentOutput, 1);
//         sleep(1000);
//         shooterMotor1.Set(ControlMode::PercentOutput, 1);
//         sleep(1000);
//         stop();
//         processing = false;
//     }
// }