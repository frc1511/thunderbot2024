#pragma once


#include <ctre/Phoenix.h>

class Shooter {
  
  ctre::phoenix::motorcontrol::can::TalonSRX shooterMotor1{10}; 
  ctre::phoenix::motorcontrol::can::TalonSRX shooterMotor2{14};  
  
  public:
    void shoot(double Power);
    void shootTop(double Power);
    void shootBottom(double Power);
    void stop();
    void food();
  
  private:
    Shooter* shooter;
};



