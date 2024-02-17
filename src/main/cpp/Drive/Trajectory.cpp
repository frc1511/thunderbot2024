// --- Drive commands ---

#include <Drive/Trajectory.h>

#define DISTANCE_THRESHOLD 2_in
#define ROTATION_THRESHOLD 1_deg

YaqoubsTrajectoryController::YaqoubsTrajectoryController() { }

YaqoubsTrajectoryController::~YaqoubsTrajectoryController() { }

/**
 * We don't have a Feedback system yet, so this does nothing
*/
void YaqoubsTrajectoryController::sendFeedback() {
    /*const char* buffer = "";
    switch (driveState) {
        case UNKNOWN:
            buffer = "unknown";
            break;
        case ACCELERATING:
            buffer = "accelerating";
            break;
        case CONSTANT:
            buffer = "constant";
            break;
        case DECELERATING:
            buffer = "decelerating";
            break;
    }
    Feedback::sendString("Yaqoub's trajectory controller", "drive state", buffer);
    Feedback::sendBoolean("Yaqoub's trajectory controller", "drive finished", driveFinished);
    Feedback::sendBoolean("Yaqoub's trajectory controller", "rotate finished", rotateFinished);
    Feedback::sendDouble("Yaqoub's trajectory controller", "state timer (seconds)", timer.Get().value());
    Feedback::sendDouble("Yaqoub's trajectory controller", "start X (meters)", start.X().value());
    Feedback::sendDouble("Yaqoub's trajectory controller", "start Y (meters)", start.Y().value());
    Feedback::sendDouble("Yaqoub's trajectory controller", "start Rotation (degrees)", start.Rotation().Degrees().value());
    Feedback::sendDouble("Yaqoub's trajectory controller", "end X (meters)", end.X().value());
    Feedback::sendDouble("Yaqoub's trajectory controller", "end Y (meters)", end.Y().value());
    Feedback::sendDouble("Yaqoub's trajectory controller", "end Rotation (degrees)", end.Rotation().Degrees().value());
    Feedback::sendDouble("Yaqoub's trajectory controller", "max velocity (mps)", maxVelocity.value());
    Feedback::sendDouble("Yaqoub's trajectory controller", "distance traveled (meters)", distanceTraveled.value());
    Feedback::sendDouble("Yaqoub's trajectory controller", "total distance (meters)", totalDistance.value());
    Feedback::sendDouble("Yaqoub's trajectory controller", "heading (degrees)", heading.value());
    Feedback::sendDouble("Yaqoub's trajectory controller", "accelerate distance (meters)", accelerateDistance.value());
    Feedback::sendDouble("Yaqoub's trajectory controller", "decelerate distance (meters)", decelerateDistance.value());
    Feedback::sendDouble("Yaqoub's trajectory controller", "constant distance (meters)", constantDistance.value());
    Feedback::sendDouble("Yaqoub's trajectory controller", "configured max acceleration", maxAcceleration.value());
    Feedback::sendDouble("Yaqoub's trajectory controller", "configured max angular velocity", maxAngularVelocity.value());
    Feedback::sendDouble("Yaqoub's trajectory controller", "configured start velocity", startVelocity.value());
    Feedback::sendDouble("Yaqoub's trajectory controller", "configured end velocity", endVelocity.value());*/
}

void YaqoubsTrajectoryController::setTrajectory(frc::Pose2d currentPose, frc::Pose2d endPose, YaqoubsTrajectoryConfig config) {
    start = currentPose;
    end = endPose;
    maxVelocity = config.maxVelocity;
    maxAcceleration = config.maxAcceleration;
    maxAngularVelocity = config.maxAngularVelocity;
    minAngularVelocity = config.minAngularVelocity;
    angularVelocityFactor = config.angularVelocityFactor;
    startVelocity = config.startVelocity;
    endVelocity = config.endVelocity;
    distanceTraveled = 0_m;
    driveFinished = false;
    rotateFinished = false;

    units::meter_t xDistance = end.X() - start.X();
    units::meter_t yDistance = end.Y() - start.Y();
    // Get the total distance to travel (a^2 + b^2 = c^2).
    totalDistance = units::math::sqrt(units::math::pow<2>(xDistance) + units::math::pow<2>(yDistance));

    /**
     * Find the maximum velocity possible.
     * 
     *  D1 = (Vm^2 - Vi^2) / 2a
     *  D2 = (-Vf^2 + Vm^2) / 2a
     *  
     *  D1 + D2 <= Dtotal
     *  
     *  Vm = sqrt((2 * a * Dtotal + Vi^2 + Vf^2) / 2)
    */
    maxVelocity = units::meters_per_second_t(std::sqrt((2 * config.maxAcceleration.value() * totalDistance.value() + std::pow(config.startVelocity.value(), 2) + std::pow(config.endVelocity.value(), 2)) / 2));

    // Clamp the maximum velocity to the configured maximum velocity.
    if (units::math::abs(maxVelocity) > units::math::abs(config.maxVelocity)) {
        maxVelocity = config.maxVelocity;
    }

    // Get the distance to accelerate to max velocity (D = (Vm^2 - Vi^2) / 2a).
    accelerateDistance = (units::math::pow<2>(maxVelocity) - units::math::pow<2>(config.startVelocity)) / (2 * config.maxAcceleration);
    // Get the distance to decelerate to final velocity (D = (Vf^2 - Vm^2) / -2a).
    decelerateDistance = (units::math::pow<2>(config.endVelocity) - units::math::pow<2>(maxVelocity)) / (2 * -config.maxAcceleration);
    // Get the remaining distance in between during which the robot is at max velocity.
    constantDistance = totalDistance - accelerateDistance - decelerateDistance;

    // Get the angle in which the robot will be moving.
    heading = units::math::atan2(yDistance, xDistance);
    
    driveState = ACCELERATING;
    timer.Reset();
    timer.Start();
}

bool YaqoubsTrajectoryController::atReference(frc::Pose2d currentPose) {
    if (driveFinished && rotateFinished) {
        return true;
    }
    
    return false;
}

frc::ChassisSpeeds YaqoubsTrajectoryController::getVelocities(frc::Pose2d currentPose) {
    if (atReference(currentPose)) {
        return {};
    }

    units::meters_per_second_t xVel = 0_mps;
    units::meters_per_second_t yVel = 0_mps;
    units::radians_per_second_t angVel = 0_rad_per_s;

    units::degree_t angle1 = end.Rotation().Degrees() - currentPose.Rotation().Degrees();
    units::degree_t angle2;

    if (end.Rotation().Degrees() > currentPose.Rotation().Degrees()) {
        angle2 = end.Rotation().Degrees() - currentPose.Rotation().Degrees() - 360_deg;
    }
    else {
        angle2 = end.Rotation().Degrees() - currentPose.Rotation().Degrees() + 360_deg;
    }
    
    // Optimize!!
    units::degree_t angleToTurn = units::math::fabs(angle1) < units::math::fabs(angle2) ? angle1 : angle2;

    if (!rotateFinished) {
        if (units::math::fabs(angleToTurn) > ROTATION_THRESHOLD) {
            // Acceleration and deceleration.
            angVel = units::degrees_per_second_t(angleToTurn.value() * angularVelocityFactor);

            if (angVel > maxAngularVelocity) {
                angVel = maxAngularVelocity;
            }
            else if (angVel < -maxAngularVelocity) {
                angVel = -maxAngularVelocity;
            }

            if (angVel < minAngularVelocity && angVel > 0_deg_per_s) {
                angVel = minAngularVelocity;
            }
            else if (angVel > -minAngularVelocity && angVel < 0_deg_per_s) {
                angVel = -minAngularVelocity;
            }
        }
        else {
            rotateFinished = true;
        }
    }

    if (!driveFinished) {
        units::meter_t xDistance = currentPose.X() - start.X();
        units::meter_t yDistance = currentPose.Y() - start.Y();

        // Get the total distance traveled (a^2 + b^2 = c^2).
        distanceTraveled = units::math::sqrt(units::math::pow<2>(xDistance) + units::math::pow<2>(yDistance));

        DriveState lastState = driveState;

        switch (driveState) {
            case UNKNOWN:
                break;
            case ACCELERATING:
                if (distanceTraveled > accelerateDistance) {
                    driveState = CONSTANT;
                }
                break;
            case CONSTANT:
                if (distanceTraveled > accelerateDistance + constantDistance) {
                    driveState = DECELERATING;
                }
                break;
            case DECELERATING:
                if (units::math::abs(totalDistance - distanceTraveled) < DISTANCE_THRESHOLD) {
                    driveFinished = true;
                    driveState = UNKNOWN;
                }
                break;
        }

        if (driveState != lastState) {
            timer.Reset();
            timer.Start();
        }

        units::meters_per_second_t vel = 0_mps;
        
        switch (driveState) {
            case UNKNOWN:
                break;
            case ACCELERATING:
                vel = (maxAcceleration * timer.Get())
                    * (std::signbit(totalDistance.value()) ? -1 : 1);
                break;
            case CONSTANT:
                vel = maxVelocity
                    * (std::signbit(totalDistance.value()) ? -1 : 1);
                break;
            case DECELERATING:
                vel = (maxVelocity - (maxAcceleration * timer.Get()))
                    * (std::signbit(totalDistance.value()) ? -1 : 1);
                break;
        }
        // Get individual X and Y components.
        xVel = vel * units::math::cos(heading);
        yVel = vel * units::math::sin(heading);
    }

    // Field-relative speeds.
    return frc::ChassisSpeeds::FromFieldRelativeSpeeds(xVel, yVel, angVel, currentPose.Rotation());
}