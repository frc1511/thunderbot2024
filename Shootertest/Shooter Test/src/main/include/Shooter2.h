#pragma once

/*
Shooter 2 is a secondary mode that strategy wanted
I don't know how it is going to work, but it needs 2 motors, not one(i think)
The motors should spin on opposite sides of the intake/shooter thing
the motors for intake aren't really affected, this is just for shooter
one of the motors should spin at 6000 rpm and the other at 4000 rpm for spin
i made it toggleable for now but idk if it will actually work
*/

#include <rev/CANSparkMax.h>

class Shooter2 {
  
  rev::CANSparkMax shooter2Motor1 {1, rev::CANSparkMax::MotorType::kBrushless}; //right side (top)
  rev::CANSparkMax shooter2Motor2 {2, rev::CANSparkMax::MotorType::kBrushless}; //left side (bottom)
  rev::CANSparkMax intakeMotor1 {3, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax intakeMotor2 {4, rev::CANSparkMax::MotorType::kBrushless};
  
  public:
    void intake(double Power);
    void stopIntake();
    void shooter2(double Power);
    void stopShooter2();
    void stop2();
    double motor1power;
    double motor2power;

    enum ShooterMode {
      DEFAULT,
      CURVED
    };

    Shooter2::ShooterMode shooterMode = Shooter2::ShooterMode::DEFAULT;
  
};



