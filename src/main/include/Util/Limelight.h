#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>
#include <wpi/SpanExtras.h>

#include <Basic/Mechanism.h>
#include <Drive/Drive.h>

class Limelight : public Mechanism {
public:
    Limelight(Drive* _drive);
    ~Limelight();

    void process() override;
    void sendFeedback() override;
    void doPersistentConfiguration() override;
    void resetToMode(MatchMode mode) override;
private:
    frc::SwerveDrivePoseEstimator<4>* poseEstimator;
    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    void updatePoseEstimator();
};