#include <GamEpiece/Shamptake.h>

Shamptake::Shamptake() {

}

Shamptake::~Shamptake() {
    
}

void Shamptake::sendFeedback() {

}

void Shamptake::doPersistentConfiguration() {
    shooterMotorRight.SetInverted(true);
}

void Shamptake::resetToMode(MatchMode mode) {
    sensorDetected = false;
    trippedBefore = false;
}

void Shamptake::intake(double Power) {
    intakeMotor1.Set(Power);
    // intakeMotor2.Set(Power);
    runIntake = true;
}
void Shamptake::stopIntake() {
    intake(0);
    runIntake = false;
}
void Shamptake::shooter(double Power) {
    double motorRightpower = 0;
    double motorLeftpower = 0;
    if (shooterMode == Shamptake::ShooterMode::CURVED) { 
        motorRightpower = Power * .95;// this should eventually be 6000 rpm
        motorLeftpower = Power * .8;// this should eventually be 4000 rpm
    } else {
        motorRightpower = Power * .8;// base speeds
        motorLeftpower = Power * .8;
        motorLeftpower = Power * .8;
    }
    shooterMotorRight.Set(motorRightpower); 
    shooterMotorLeft.Set(motorLeftpower); 
}
void Shamptake::stop() {
    intake(0);
    shooter(0);
}
void Shamptake::shooterSwitch() {
   
    if (shooterMode == Shamptake::ShooterMode::DEFAULT) {
        shooterMode = Shamptake::ShooterMode::CURVED;
    } else if (shooterMode == Shamptake::ShooterMode::CURVED) {
        shooterMode = Shamptake::ShooterMode::DEFAULT;
    }
}
void Shamptake::process() {
    //if no snesor detected
        //run intake (Before Sensor) (IntakeSpeed = Normal)
        //except if intake previously tripped then stop (Now past sensor) (IntakeSpeed = Stop)
            //but if shooting, intake max (Now leaving) (IntakeSpeed = Shooting) (Handled in controls)
    //if sensor detected (In Sensor range)
        //slow intake (IntakeSpeed = Slow)
        //intake previously tripped = true (So we stop when past)
    //if outtaking
        //Outtake (Handled in controls)
        //intake previously tripped = false (So we don't stop when inkaing next time)
    sensorDetected = !noteSensor.Get();
    
    if (!sensorDetected) {
        if (trippedBefore) { // Past Sensor
            intakeSpeed = IntakeSpeed::STOP;
        } else { // Before Sensor
            intakeSpeed = IntakeSpeed::NORMAL;
        }
    } else { // Sensor Tripped
        intakeSpeed = IntakeSpeed::SLOW;
        trippedBefore = true;
        printf("SET\n");
    }
}
void Shamptake::runIntakeMotors() {
    double speed = 0;
    switch (intakeSpeed)
    {
    case IntakeSpeed::NORMAL:
        printf("NORMAL\n");
        speed = 0.7;
        break;
    case IntakeSpeed::STOP:
        printf("STOP\n");
        speed = 0;
        break;
    case IntakeSpeed::SLOW:
        printf("SLOW\n");
        speed = 0.5;
        break;
    case IntakeSpeed::FIRE:
        printf("FIRE\n");
        speed = 0.8;
        break;
    case IntakeSpeed::OUTTAKE:
        printf("OUTTAKE\n");
        speed = -0.4;
        break;
    default:
        printf("[SHAMPTAKE] IntakeSpeed was not set properly!");
        speed = 0;
        break;
    }
    intake(speed);
}