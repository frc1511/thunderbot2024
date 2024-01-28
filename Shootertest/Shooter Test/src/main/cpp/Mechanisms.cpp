#pragma once

#include <Robot.h>


void Mechanisms::ShooterSwitch() {
    if (shooter2->shooterMode == Shooter2::ShooterMode::DEFAULT) {
        shooter2->shooterMode = Shooter2::ShooterMode::CURVED;
    } else if (shooter2->shooterMode == Shooter2::ShooterMode::CURVED) {
        shooter2->shooterMode = Shooter2::ShooterMode::DEFAULT;
    }
}

void Mechanisms::Init(Shooter2* ptrShooter2) {
    shooter2 = ptrShooter2;
}