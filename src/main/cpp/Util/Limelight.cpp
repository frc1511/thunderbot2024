#include <Util/Limelight.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/estimator/PoseEstimator.h>
#include <Util/LimeLightHelpers.h>

Limelight::Limelight(Drive* _drive)
: poseEstimator(_drive->getPoseEstimator()) {
    
}

Limelight::~Limelight() {
    
}

void Limelight::doPersistentConfiguration() {
}

void Limelight::updatePoseEstimator() {
    if (!limelightEnabled) return;
    LimelightHelpers::PoseEstimate mt1 = LimelightHelpers::getBotPoseEstimate_wpiBlue("limelight");

    if (mt1.tagCount == 0) return;

    if (mt1.tagCount == 1 && mt1.rawFiducials.size() == 1) {
        if (mt1.rawFiducials[0].ambiguity > .7) {
            return;
        }
        if (mt1.rawFiducials[0].distToCamera > 3) {
            return;
        }
    }

    for (size_t i = 0; i < mt1.rawFiducials.size(); i++) {
        if (mt1.rawFiducials[i].ambiguity > .7) {
            return;
        }
    }

    if (poseEstimator != nullptr) {
        poseEstimator->AddVisionMeasurement(mt1.pose, mt1.timestampSeconds);
    } else {
        printf("LIMELIGHT: poseEstimator is nullptr\n");
    }
}

void Limelight::sendFeedback() {
    frc::SmartDashboard::PutNumber("Limelight_targetOffsetAngle_Horizontal", table->GetNumber("tx", 0.0));
    frc::SmartDashboard::PutNumber("Limelight_targetOffsetAngle_Vertical", table->GetNumber("ty", 0.0));
    frc::SmartDashboard::PutNumber("Limelight_targetArea", table->GetNumber("ta", 0.0));
    frc::SmartDashboard::PutNumber("Limelight_targetSkew", table->GetNumber("ts", 0.0));
}

void Limelight::resetToMode(MatchMode mode) {

}

void Limelight::process() {
    updatePoseEstimator();
}