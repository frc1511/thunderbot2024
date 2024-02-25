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
    if (scoreAnimation) {
        double percent = scoreAnimationTimer.Get() / 0.25_s;
        if (percent >= 1.0) {
            scoreAnimation = false;
            scoreAnimationTimer.Stop();
        }
        else {
            setColor(frc::Color::kRed);
            int end = static_cast<int>(percent * LED_TOTAL);
            int start = std::clamp(end - 10.0, 0.0, (double)LED_TOTAL);

            for (int i = start; i < end; i++) {
                setMirroredPixel(i, frc::Color::kYellow);
            }
        }
    }

    if (!scoreAnimation) {
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
                    setPixel(i, arm->isOnLowerLimit() ? frc::Color::kGreen : frc::Color::kRed);
                double armPct = arm->getBoreNormalizedPosition();
                size_t end = std::max((size_t)4, (size_t)(4 * armPct));
                for (size_t i = 0; i < 5; i++)
                    setPixel(18 + i, (i <= end) ? frc::Color::kGreen : frc::Color::kRed);
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
            case LEDMode::HAS_GAMEPIECE:
                setColor(frc::Color::kRed);
                break;
            case LEDMode::FIRE:
                fire();
                break;
            case LEDMode::ALLIANCE:
                setColor(frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue ? frc::Color::kBlue : frc::Color::kRed);
                break;
            case LEDMode::SOURCE:
                setColor(frc::Color::kPurple);
                break;
            case LEDMode::CRATER_MODE:
                setColor(frc::Color::kGreen);
                break;
            case LEDMode::CALIBRATING:
                setColor(frc::Color::kCornflowerBlue);
                break;
            case LEDMode::HOME_DEPOT:
                setColor(frc::Color(255, 27, 0));
                break;
            case LEDMode::KNIGHT_RIDER:
                kitt();
                break;
            case LEDMode::HANG_MODE:
                rainbow();
                break;
            case LEDMode::PARTY:
                party();
                break;
            case LEDMode::CUSTOM:
                setColor(customColor);
                break;
        }
    }

    strip.SetData(stripBuffer);

    rainbowOffset -=- 1;
    rainbowOffset %= 180;

    // Hi Trevor!
}

void BlinkyBlinky::setLEDMode(LEDMode mode) {
    ledMode = mode;
}

void BlinkyBlinky::setCustomColor(frc::Color color) {
    customColor = color;
}

void BlinkyBlinky::playScoreAnimation() {
    scoreAnimation = true;
    scoreAnimationTimer.Reset();
    scoreAnimationTimer.Start();
}

void BlinkyBlinky::setPixel(std::size_t index, frc::Color color) {
    stripBuffer.at(index).SetLED(color);
}

void BlinkyBlinky::setMirroredPixel(std::size_t index, frc::Color color) {
    setPixel(index, color);
    setPixel(LED_TOTAL - 1 - index, color);
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

void BlinkyBlinky::kitt() {
    kittIter += kittDir;

    // Reverse direction when reaches end.
    if (kittIter >= KITT_LOOPS || kittIter <= 0) {
        kittDir = -kittDir;
    }

    double percent = kittIter / KITT_LOOPS;

    int pixel = static_cast<int>(percent * (LED_TOTAL - 1));

    double fadeRange = LED_TOTAL * 0.3;

    setColor(frc::Color::kBlack);

    // Match alliance color.
    int hueOffset = 0;
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
        hueOffset = 120;
    }
    else if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
        hueOffset = 45;
    }

    // Fade up.
    for (int i = pixel; i <= (pixel + fadeRange > LED_TOTAL - 1 ? LED_TOTAL - 1 : pixel + fadeRange); i++) {
        double percent = static_cast<double>(i - pixel) / fadeRange;

        double value = (1 / percent) * 128;

        setMirroredPixel(i, frc::Color::FromHSV(hueOffset + percent * 5, value * 0.5 + 192, value));
    }
    // Fade down.
    for (int i = pixel; i >= (pixel - fadeRange < 0 ? 0 : pixel - fadeRange); i--) {
        double percent = static_cast<double>(pixel - i) / fadeRange;

        double value = (1 / percent) * 128;

        setMirroredPixel(i, frc::Color::FromHSV(hueOffset + percent * 5, value * 0.5 + 192, value));
    }

    // Middle.
    setMirroredPixel(pixel, frc::Color::FromHSV(hueOffset, 255, 255));
}

void BlinkyBlinky::fire() {
    fireIter += fireDir;

    if (fireIter >= fireLoops || fireIter <= 0) {
        fireDir = -fireDir;
        fireLoops = FIRE_MAX_LOOPS * ((60 + (static_cast<double>(rand() % 40))) / 100.0);
        fireRange = (LED_TOTAL - 1) * ((50 + (static_cast<double>(rand() % 51))) / 100.0);
        if (fireDir == 1) {
            fireIter = fireLoops;
        }
        else {
            fireIter = 0;
        }
    }

    double percent = fireIter / fireLoops;
    percent = std::clamp(percent, 0.0, 1.0);

    // int pixel = static_cast<int>(percent * fireRange);
    int pixel = fireRange;

    for (int i = 0; i < pixel; i++) {
        setMirroredPixel(i, frc::Color::FromHSV((static_cast<double>(i) / (LED_TOTAL - 1)) * 5.0, 255 /*(1.0 / (static_cast<double>(i) / pixel)) * 255*/, 128));
    }
    for (int i = pixel; i < LED_TOTAL; i++) {
        setMirroredPixel(i, frc::Color::kBlack);
    }
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
        case LEDMode::RAINBOW:
            modeString = "rainbow";
            break;
        case LEDMode::SOURCE:
            modeString = "source";
            break;
        case LEDMode::HAS_GAMEPIECE:
            modeString = "has gamepiece";
            break;
        case LEDMode::FIRE:
            modeString = "fire";
            break;
        case LEDMode::ALLIANCE:
            modeString = "alliance";
            break;
        case LEDMode::CRATER_MODE:
            modeString = "crater mode";
            break;
        case LEDMode::CALIBRATING:
            modeString = "calibrating";
            break;
        case LEDMode::HOME_DEPOT:
            modeString = "home depot";
            break;
        case LEDMode::KNIGHT_RIDER:
            modeString = "knight rider";
            break;
        case LEDMode::HANG_MODE:
            modeString = "hang mode";
            break;
        case LEDMode::PARTY:
            modeString = "party";
            break;
        case LEDMode::CUSTOM:
            modeString = "custom";
            break;
    }


    frc::SmartDashboard::PutString("BlinkyBlinky_Mode", modeString);
    frc::SmartDashboard::PutNumber("BlinkyBlinky_RainbowOffset", rainbowOffset);
}
