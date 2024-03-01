#pragma once

#include <Basic/Mechanism.h>
#include <Basic/IOMap.h>
#include <frc/AddressableLED.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <array>
#include <map>

#include "Hanger/Hang.h"
#include "GamEpiece/Arm.h"
#include "GamEpiece/Shamptake.h"

#define LED_TOTAL 39



class BlinkyBlinky : public Mechanism {
public:
    BlinkyBlinky(Hang *hang, Arm *arm, Shamptake *shamptake);
    ~BlinkyBlinky();

    void sendFeedback() override;
    void resetToMode(MatchMode mode) override;
    void process() override;

    enum class LEDMode {
        OFF,
        PIT_MODE,
        RAINBOW,
        ALLIANCE,
        SOURCE,
        INTAKE,
        HAS_GAMEPIECE,
        SCORE,
        AMP,
        BASE,
        CALIBRATING,
        HANG_MODE,
        PARTY,
    };

    void setLEDMode(LEDMode mode);



    void playScoreAnimation();

private:
    frc::AddressableLED strip {PWM_BLINKY_BLINKY };

    std::array<frc::AddressableLED::LEDData, LED_TOTAL> stripBuffer;

    void setPixel(std::size_t index, frc::Color color);

    void setColor(frc::Color color);

    void interpolateHue(int lowHue, int highHue, int offset);

    void rainbow();

    void party();

    Hang *hang;
    Arm *arm;
    Shamptake *shamptake;
    LEDMode ledMode = LEDMode::ALLIANCE;

    int rainbowOffset = 0;
    int strobeIter = 0;
    bool strobeOn = true;


    bool scoreAnimation = false;
    frc::Timer scoreAnimationTimer;
};