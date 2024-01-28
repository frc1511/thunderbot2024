#pragma once

#include <Mechanisms.h>

void ShooterSwitch() {
    if (shooterMode == 0) {
        shooterMode ++;
    } else if (shooterMode == 1) {
        shooterMode = 0;
    }
    
}