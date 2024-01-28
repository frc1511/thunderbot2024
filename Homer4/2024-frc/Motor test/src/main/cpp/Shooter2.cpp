#pragma once

/*
Shooter 2 is a secondary mode that strategy wanted
I don't know how it is going to work, but it needs 2 motors, not one(i think)
The motors should spin on opposite sides of the intake/shooter thing
the motors for intake aren't really affected, this is just for shooter
one of the motors should spin at 6000 rpm and the other at 4000 rpm for spin
i made it toggleable for now but idk if it will actually work
*/

#include <Shooter2.h>
#include <Shooter.h> //this is for intake stuff

void Shooter2::intake(double Power) {
    feederMotor.Set(Power);
}
void Shooter2::stopIntake() {
    intake(0);
}
void Shooter2::shooter2(double Power) {
    intakeMotor1.Set(Power);
    intakeMotor2.Set(Power);
    shooter2Motor1.set(Power * .6); // this should eventually be 6000 rpm
    shooter2Motor2.set(Power * .4); // this should eventually be 4000 rpm
}
void Shooter2::stopShooter2() {
    shooter2(0);
}
void Shooter2::stop2() {
    intake(0);
    shooter2(0);
}

Shooter2::Shooter() { // idk where this is called so im leaving it as it was before
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