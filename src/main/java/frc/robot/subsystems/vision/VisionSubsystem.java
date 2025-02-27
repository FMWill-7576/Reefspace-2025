package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.eUtil;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

/*
 * Author: @knkr1
 * FMWILL
 * #7576
 */

public class VisionSubsystem extends SubsystemBase {
    PhotonCamera camera;
    Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonPoseEstimator photonPoseEstimator;
    double final_kP = VisionConstants.kP;
    double final_kI = VisionConstants.kI;
    double final_kD = VisionConstants.kD;

    public VisionSubsystem() {
        camera = new PhotonCamera("cam7576");
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)); // facing forward,
                                                                                                 // 0.5meter forward of
                                                                                                 // center, 0.5 meter up
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                robotToCam);
    }

    public PIDController getLastestPID() {
        PIDController s = new PIDController(Preferences.getDouble(VisionConstants.kP_string, final_kP),
                Preferences.getDouble(VisionConstants.kP_string, final_kI),
                Preferences.getDouble(VisionConstants.kP_string, final_kD));
        return s;
    }

    public double getAprilTagYawAsRadian(int id) {
        Pose3d pose = aprilTagFieldLayout.getTagPose(id).get();
        return pose.getRotation().getAngle();
    }

    public PhotonTrackedTarget bestTarget() {
        List<PhotonPipelineResult> result = camera.getAllUnreadResults();
        return result.get(0).getBestTarget();
    }

    public void forwardYawDrive(SwerveSubsystem swerve, CommandPS5Controller controller) {
        boolean targetVisible = false;
        double targetYaw = 0.0;
        double targetRange = 0.0;
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (eUtil.isIntExistsInArray(target.getFiducialId(), VisionConstants.reefIDs)) {
                        // Found Tag 7, record its information
                        targetYaw = target.getYaw();
                        targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                                0.5, // Measured with a tape measure, or in CAD.
                                VisionConstants.aprilHeightInfo(target.getFiducialId()), // From 2024 game manual for ID
                                                                                         // 7
                                Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
                                Units.degreesToRadians(target.getPitch()));

                        targetVisible = true;
                    }
                }
            }
        }
        if (targetVisible == true) {
            PIDController forwardPID = new PIDController(1, 0, 0);
            PIDController turnPID = new PIDController(1, 0, 0);
            double forward = forwardPID.calculate(targetRange, 0) * -1 * 0.2;
            double turn = turnPID.calculate(targetYaw, 0) * -1 * 0.2;
            swerve.driveCommand(
                    () -> forward,
                    () -> controller.getLeftX(),
                    () -> turn);
            turnPID.close();
            forwardPID.close();
        }
    }

    public void yawDrive(SwerveSubsystem swerve, CommandPS5Controller controller) {
        // Read in relevant data from the Camera
        boolean targetVisible = false;
        double targetYaw = 0.0;
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (eUtil.isIntExistsInArray(target.getFiducialId(), VisionConstants.reefIDs)) {
                        targetYaw = target.getYaw();
                        targetVisible = true;
                        break;
                    }
                }
            }
        }
        if (targetVisible) {
            PIDController turnPID = new PIDController(1, 0, 0);
            double turn = turnPID.calculate(targetYaw, swerve.getPose().getRotation().getRadians());
            swerve.driveCommand(
                    () -> controller.getLeftY() * 0.25,
                    () -> controller.getLeftX() * -1,
                    () -> MathUtil.clamp(turn,0,1)*0.2);
            turnPID.close();
        }
    }

    public boolean IsAtDesiredYaw(double desired,double actual){
        return MathUtil.isNear(desired, actual, 0.1);
    }

    @Override
    public void periodic() {

    }
}
