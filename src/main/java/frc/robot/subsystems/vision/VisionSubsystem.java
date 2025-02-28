package frc.robot.subsystems.vision;

import java.lang.reflect.Constructor;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants;
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

    double aprilyaw = 0;
    boolean isApril = false;

    PIDController turnPID = new PIDController(0.1, 0, 0.02);
    PIDController forwardPID = new PIDController(1, 0, 0);

    public VisionSubsystem() {
        camera = new PhotonCamera("Limelight");
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        robotToCam = new Transform3d(
                new Translation3d(0, eUtil.centimeterToMeter((195 / 10) + 44 / 10), eUtil.centimeterToMeter(290 / 10)),
                new Rotation3d(0, Math.toRadians(2), Math.toRadians(18))); // facing forward,
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
            double forward = forwardPID.calculate(targetRange, 0) * -1 * 0.2;
            double turn = turnPID.calculate(targetYaw-VisionConstants.yaw, 0) * -1 * 0.2;
            swerve.driveCommand(
                    () -> forward,
                    () -> controller.getLeftX(),
                    () -> turn);
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        var results = camera.getAllUnreadResults();
        return photonPoseEstimator.update(results.get(results.size() - 1));
    }

    public Command yawDrive(SwerveSubsystem swerve, CommandPS5Controller controller) {
        return swerve.driveFieldOriented(
                SwerveInputStream.of(swerve.getSwerveDrive(),
                        () -> controller.getLeftY(),
                        () -> controller.getLeftX())
                        .withControllerRotationAxis(() -> MathUtil.clamp(turnPID.calculate(aprilyaw, 0), -1, 1)
                                * 0.25 * -1)
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(Constants.OperatorConstants.swerveSpeed));
    }

    public boolean IsAtDesiredYaw(double desired, double actual) {
        return MathUtil.isNear(desired, actual, 0.1);
    }

    public boolean isAprilOnResult() {
        return isApril;
    }

    @Override
    public void periodic() {
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 12) {
                        aprilyaw = target.getYaw();
                        isApril = true;
                        break;
                    }
                }
            } else {
                isApril = false;
                aprilyaw = 0;
            }
        }
    }
}
