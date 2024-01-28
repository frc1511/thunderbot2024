#pragma once

class Mechanisms {
public:
    void ShooterSwitch();
    void Init(Shooter2* ptrShooter2);
private:
    Shooter2* shooter2;
};
