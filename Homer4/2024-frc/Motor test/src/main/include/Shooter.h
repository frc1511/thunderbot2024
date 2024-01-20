#pragma once


// #include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>

class Shooter {
  
  rev::CANSparkMax feederMotor {1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax intakeMotor1 {3, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax intakeMotor2 {4, rev::CANSparkMax::MotorType::kBrushless};
  
  public:
    void intake(double Power);
    void stopIntake();
    void shooter(double Power);
    void stopShooter();
    void stop();
    Shooter ();
  
  private:
  
};



