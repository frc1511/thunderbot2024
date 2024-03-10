#pragma once

#include <Basic/Mechanism.h>
#include <Drive/Drive.h>
#include <GamEpiece/Shamptake.h>
#include <GamEpiece/Arm.h>
#include <frc/Timer.h>
#include <Drive/CSVTrajectory.h>
#include <Autonomous/Action.h>
#include <frc/smartdashboard/SendableChooser.h>

#define DEPLOY_DIR "/home/lvuser/deploy/"

class Auto : public Mechanism {

public:
    Auto(Drive *drive, Shamptake *shamptake, Arm *arm);

    void process() override;
    void sendFeedback() override;
    void resetToMode(MatchMode mode) override;

    frc::SendableChooser<int> autoSelector;
    void autoSelectorInit();

private:
    enum class AutoMode
    {
        DO_NOTHING   = 0,
        SPEAKER_1_GP = 1,
        SPEAKER_2_GP = 2,
        HAVOC        = 3,
        BASIC_LOC_1  = 4,
        BASIC_LOC_2  = 5,
        BASIC_LOC_3  = 6,
        SQUARE       = 7,
        TEST         = 8
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
    void havoc();
    void basic_loc_1();
    void basic_loc_2();
    void basic_loc_3();
    void squareTest();

    const std::map<AutoMode, const char*> autoModeNames {
        { AutoMode::DO_NOTHING,     "Do Nothing"        },
        { AutoMode::SPEAKER_1_GP,   "1 GP Speaker"      },
        { AutoMode::SPEAKER_2_GP,   "2 GP Speaker"      },
        { AutoMode::HAVOC,          "Hail Mary"         },
        { AutoMode::BASIC_LOC_1,    "Shamptake Loc 1"   },
        { AutoMode::BASIC_LOC_2,    "Shamptake Loc 2"   },
        { AutoMode::BASIC_LOC_3,    "Shamptake Loc 3"   },
        { AutoMode::SQUARE,         "Square Test"       },
        { AutoMode::TEST,           "Test"              }

    };
    int step = 0;

    enum class Path {
        SPEAKER_1,
        SPEAKER_2_STAGE_2,
        HAVOC,
        BASIC_LOC_1,
        BASIC_LOC_2,
        BASIC_LOC_3,
        SQUARE
    };
    const std::map<Path, CSVTrajectory> bluePaths {
        { Path::SPEAKER_1,          CSVTrajectory{ DEPLOY_DIR "ThunderAuto/speaker_1.csv",          false } },
        { Path::SPEAKER_2_STAGE_2,  CSVTrajectory{ DEPLOY_DIR "ThunderAuto/speaker_2_stage_2.csv",  false } },
        { Path::HAVOC,              CSVTrajectory{ DEPLOY_DIR "ThunderAuto/havoc.csv",              false } },
        { Path::BASIC_LOC_1,        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/basic_pickup_1.csv",     false } },
        { Path::BASIC_LOC_2,        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/basic_pickup_2.csv",     false } },
        { Path::BASIC_LOC_3,        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/basic_pickup_3.csv",     false } },
        { Path::SQUARE,             CSVTrajectory{ DEPLOY_DIR "ThunderAuto/square_test.csv",        false } }

    };
    const std::map<Path, CSVTrajectory> redPaths {
        { Path::SPEAKER_1,          CSVTrajectory{ DEPLOY_DIR "ThunderAuto/speaker_1.csv",          true  } },
        { Path::SPEAKER_2_STAGE_2,  CSVTrajectory{ DEPLOY_DIR "ThunderAuto/speaker_2_stage_2.csv",  true  } },
        { Path::HAVOC,              CSVTrajectory{ DEPLOY_DIR "ThunderAuto/havoc.csv",              true  } },
        { Path::BASIC_LOC_1,        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/basic_pickup_1.csv",     true  } },
        { Path::BASIC_LOC_2,        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/basic_pickup_2.csv",     true  } },
        { Path::BASIC_LOC_3,        CSVTrajectory{ DEPLOY_DIR "ThunderAuto/basic_pickup_3.csv",     true  } },
        { Path::SQUARE,             CSVTrajectory{ DEPLOY_DIR "ThunderAuto/square_test.csv",        true  } }
    };
    const std::map<Path, CSVTrajectory>* paths = nullptr;

    std::map<u_int32_t, Action*> actions {

    };

    std::string autoSelected;
};