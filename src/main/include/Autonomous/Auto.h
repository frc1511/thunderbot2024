#pragma once

#include <Basic/Mechanism.h>
#include <Drive/Drive.h>
#include <GamEpiece/Shamptake.h>
#include <GamEpiece/Arm.h>
#include <frc/Timer.h>
#include <Drive/CSVTrajectory.h>
#include <Autonomous/Action.h>

#define DEPLOY_DIR "/home/lvuser/deploy/"

class Auto : public Mechanism {

public:
    Auto(Drive *drive, Shamptake *shamptake, Arm *arm);

    void process() override;
    void sendFeedback() override;
    void resetToMode(MatchMode mode) override;

private:
    enum class AutoMode
    {
        DO_NOTHING   = 0,
        SPEAKER_1_GP = 1,
        SPEAKER_2_GP = 2,
        TEST         = 3,
    };
    Drive *drive;
    Shamptake *shamptake;
    Arm *arm;

    Auto::AutoMode mode = AutoMode::DO_NOTHING;

    frc::Timer delayTimer,
               autoTimer;

    void doNothing();
    void test();
    void speaker1();
    void speaker2();
    const std::map<AutoMode, const char*> autoModeNames {
        { AutoMode::DO_NOTHING,     "Do Nothing"        },
        { AutoMode::SPEAKER_1_GP,   "1 GP Speaker"      },
        { AutoMode::SPEAKER_2_GP,   "2 GP Speaker"      },
        { AutoMode::TEST,           "Test"              },

    };
    int step = 0;

    enum class Path {
        SPEAKER_1,
        SPEAKER_2_STAGE_2,
    };
    const std::map<Path, CSVTrajectory> bluePaths {
        { Path::SPEAKER_1,          CSVTrajectory{ DEPLOY_DIR "ThunderAuto/speaker_1.csv",          false } },
        { Path::SPEAKER_2_STAGE_2,  CSVTrajectory{ DEPLOY_DIR "ThunderAuto/speaker_2_stage_2.csv",  false } }
    };
    const std::map<Path, CSVTrajectory> redPaths {
        { Path::SPEAKER_1,          CSVTrajectory{ DEPLOY_DIR "ThunderAuto/speaker_1.csv",          true  } },
        { Path::SPEAKER_2_STAGE_2,  CSVTrajectory{ DEPLOY_DIR "ThunderAuto/speaker_2_stage_2.csv",  true  } },
    };
    const std::map<Path, CSVTrajectory>* paths = nullptr;

    std::map<u_int32_t, Action*> actions {

    };
};