#pragma once

#include <Shooter.h>


void Shooter::shoot(double Power) {
        shooterMotor1.Set(ControlMode::PercentOutput, Power);
        shooterMotor2.Set(ControlMode::PercentOutput, Power);
}

void Shooter::shootTop(double Power) {
        shooterMotor2.Set(ControlMode::PercentOutput, Power);
}

void Shooter::shootBottom(double Power) {
        shooterMotor1.Set(ControlMode::PercentOutput, Power);
}

void Shooter::stop() {
    shoot(0);
}

void Shooter::food() {
    static bool processing = false;
    if (!processing) {
        processing = true;
        shooterMotor2.Set(ControlMode::PercentOutput, 1);
        sleep(1000);
        shooterMotor1.Set(ControlMode::PercentOutput, 1);
        sleep(1000);
        stop();
        processing = false;
    }
}