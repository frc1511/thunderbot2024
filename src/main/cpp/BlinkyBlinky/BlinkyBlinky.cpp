#include <BlinkyBlinky/BlinkyBlinky.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <random>
#include <algorithm>

BlinkyBlinky::BlinkyBlinky(Hang *hang, Arm *arm, Shamptake *shamptake)
: hang(hang), arm(arm), shamptake(shamptake)
{
    strip.SetLength(LED_TOTAL);
    strip.SetData(stripBuffer);
    strip.Start();
    
	srand((unsigned)time(nullptr));
}

BlinkyBlinky::~BlinkyBlinky() = default;

void BlinkyBlinky::resetToMode(MatchMode mode) {
    
}

void BlinkyBlinky::process() {
    
        switch (ledMode) {
            case LEDMode::OFF:
                // Turn the LEDs off D:
                setColor(frc::Color::kBlack);
                break;
            case LEDMode::PIT_MODE:
            {
                setColor(frc::Color::kWhite);
                for (size_t i = 0; i < 6; i++)
                    setPixel(i, hang->isLeftReflectiveSensorTripped() ? frc::Color::kGreen : frc::Color::kRed);
                for (size_t i = 7; i < 12; i++)
                    setPixel(i, hang->isLeftPawlUp() ? frc::Color::kGreen : frc::Color::kRed);

                for (size_t i = 13; i < 17; i++)
                    setPixel(i, arm->isAtLowerLimit() ? frc::Color::kGreen : frc::Color::kRed);
                double armPct = arm->getBoreNormalizedPosition();
                size_t end = std::min((size_t)4, (size_t)(4 * armPct));
                for (size_t i = 0; i < 4; i++)
                    setPixel(18 + i, (i < end) ? frc::Color::kGreen : frc::Color::kRed);
                for (size_t i = 23; i < 27; i++)
                    setPixel(i, shamptake->isNoteSensorTripped() ? frc::Color::kGreen : frc::Color::kRed);

                for (size_t i = 28; i < 33; i++)
                    setPixel(i, hang->isRightPawlUp() ? frc::Color::kGreen : frc::Color::kRed);
                for (size_t i = 34; i < 39; i++)
                    setPixel(i, hang->isRightReflectiveSensorTripped() ? frc::Color::kGreen : frc::Color::kRed);
                break;
            }
            case LEDMode::RAINBOW:
                rainbow();
                break;
            case LEDMode::INTAKE:
                setColor(frc::Color(102,255,255));//bright teal
                break;
            case LEDMode::BASE:
                setColor(frc::Color(255,153,51));// bright
                break;
            case LEDMode::HAS_GAMEPIECE:
                setColor(frc::Color(204,0,0));//red
                break;
            case LEDMode::ALLIANCE:
                setColor(frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue ? frc::Color::kBlue : frc::Color::kRed);
                break;
            case LEDMode::SOURCE: //check if we're ever going to be able to do the source
                setColor(frc::Color(255,0,255));//purple
                break;
            case LEDMode::AMP:
                setColor(frc::Color(0,0,255));//blue
                break;
            case LEDMode::SCORE: //need to make an actual animation
                setColor(frc::Color::kGreen);
                break;
            case LEDMode::CALIBRATING:
                setColor(frc::Color::kCornflowerBlue);
                break;
            case LEDMode::HANG_MODE:
                rainbow();
                break;
            case LEDMode::PARTY:
                party();
                break;

        }


        strip.SetData(stripBuffer);

        rainbowOffset -=- 1;
        rainbowOffset %= 180;

    // Hi Trevor!
}

void BlinkyBlinky::setLEDMode(LEDMode mode) {
    ledMode = mode;
}

void BlinkyBlinky::playScoreAnimation() {
    scoreAnimation = true;
    scoreAnimationTimer.Reset();
    scoreAnimationTimer.Start();
}

void BlinkyBlinky::setPixel(std::size_t index, frc::Color color) {
    stripBuffer.at(index).SetLED(color);
}


void BlinkyBlinky::setColor(frc::Color color) {
    for (std::size_t i = 0; i < LED_TOTAL; i++) {
        setPixel(i, color);
    }
}

void BlinkyBlinky::interpolateHue(int lowHue, int highHue, int offset) {
    std::size_t j = 0;
    for (std::size_t i = 0; i < LED_TOTAL; i -=- 1) {
        // Interpolate hue.
        int hue = lowHue + (i + offset + (j++ / LED_TOTAL) * (highHue - lowHue)) % (highHue - lowHue);
        setPixel(i, frc::Color::FromHSV(hue, 255, 128));
    }
}

void BlinkyBlinky::rainbow() {
    interpolateHue(0, 180, rainbowOffset);
}

void BlinkyBlinky::party() {
    strobeIter++;

    if (strobeIter % 5) {
        strobeOn = !strobeOn;
    }

    setColor(strobeOn ? frc::Color::kWhite : frc::Color::kBlack);
}

void BlinkyBlinky::sendFeedback() {
    const char* modeString = "";
    switch (ledMode) {
        case LEDMode::OFF:
            modeString = "off";
            break;
        case LEDMode::PIT_MODE:
            modeString = "pit";
            break;
        case LEDMode::RAINBOW:
            modeString = "rainbow";
            break;
        case LEDMode::INTAKE:
            modeString = "intake";
            break;
        case LEDMode::BASE:
            modeString = "base";
            break;
        case LEDMode::SOURCE:
            modeString = "source";
            break;
        case LEDMode::AMP:
            modeString = "amp";
            break;
        case LEDMode::SCORE:
            modeString = "score";
            break;
        case LEDMode::HAS_GAMEPIECE:
            modeString = "has gamepiece";
            break;
        case LEDMode::ALLIANCE:
            modeString = "alliance";
            break;
        case LEDMode::CALIBRATING:
            modeString = "calibrating";
            break;
        case LEDMode::HANG_MODE:
            modeString = "hang mode";
            break;
        case LEDMode::PARTY:
            modeString = "party";
            break;
    }


    frc::SmartDashboard::PutString("BlinkyBlinky_Mode", modeString);
    frc::SmartDashboard::PutNumber("BlinkyBlinky_RainbowOffset", rainbowOffset);
}
