#pragma once

#include <Drive.h>

void Drive::move() {
          //get axis
double leftYAxis = 1 * driveController.GetRawAxis(1); //left side tank movement
leftYAxis = axisProccess(leftYAxis);
  leftDriveMotor1.Set(leftYAxis);
  leftDriveMotor2.Set(leftYAxis);

double rightYAxis = -1 * driveController.GetRawAxis(5); //right side tank movement
rightYAxis = axisProccess(rightYAxis);
  rightDriveMotor1.Set(rightYAxis);
  rightDriveMotor2.Set(rightYAxis);

}

double Drive::axisProccess(double rawAxis) { //procces the speed and stuff 
    if (rawAxis > 0.2){//deadband
        rawAxis -= 0.2;//deadband
        rawAxis /= 0.2; //not deadband
    }
    else if (rawAxis < -0.2){ //deadband
        rawAxis += 0.2;//deadband
        rawAxis /= 0.8; //not deadband
    }
    else{
        rawAxis = 0;
    }
    return rawAxis;
}