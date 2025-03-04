package frc.robot.subsystems.vision;

import java.lang.annotation.Target;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants;
import frc.robot.eUtil;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.led.LedSubsystem;
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
    PhotonTrackedTarget currentTarget;
    double final_kP = VisionConstants.kP;
    double final_kI = VisionConstants.kI;
    double final_kD = VisionConstants.kD;

    double aprilyaw = 0;
    boolean isValidApril = false;
    boolean isApril = false;

    PIDController turnPID = new PIDController(0.1, 0, 0.02);
    PIDController forwardPID = new PIDController(1, 0, 0);

    public VisionSubsystem() {

        camera = new PhotonCamera("Limelight");
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
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

    public double getAprilDistance() {
        if (isApril && isValidApril) {
            return currentTarget.getBestCameraToTarget().getX();
        } else {
            return 99.0;
        }

    }

    public boolean isAprilOnSight() {
        return isValidApril;
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        var results = camera.getAllUnreadResults();
        return photonPoseEstimator.update(results.get(results.size() - 1));
    }

    public double getAprilY() {
        if (isApril && isValidApril) {
            return currentTarget.getBestCameraToTarget().getY();
        } else {
            return 99.0;
        }
    }

    public double getAprilX() {
        if (isApril && isValidApril) {
            return currentTarget.getBestCameraToTarget().getX();
        } else {
            return 99.0;
        }
    }

    public double getAprilZ() {
        if (isApril && isValidApril) {
            return currentTarget.getBestCameraToTarget().getRotation().getAngle();
        } else {
            return 99.0;
        }
    }

    public Command yawDrive(SwerveSubsystem swerve, CommandPS5Controller controller) {
        double desiredYaw = aprilyaw + VisionConstants.yaw;
        if (isApril) {
            return swerve.driveFieldOriented(
                    SwerveInputStream.of(swerve.getSwerveDrive(),
                            () -> controller.getLeftY(),
                            () -> controller.getLeftX())
                            .withControllerRotationAxis(() -> MathUtil.clamp(turnPID.calculate(desiredYaw, 0), -1, 1)
                                    * 0.25 * -1)
                            .deadband(OperatorConstants.DEADBAND)
                            .scaleTranslation(1));
        }
        return Commands.run(() -> {
        });
    }

    public Command allignDrive(SwerveSubsystem swerve, CommandPS5Controller controller) {
        double goal = 163;
        return run(() -> swerve.drive(
                swerve.getTargetSpeeds(
                        controller.getLeftX() * -1 * 0.5,
                        controller.getLeftY() * 0.5,
                        Rotation2d.fromDegrees(goal))));

    }

    public Command allignDataDrive(SwerveSubsystem swerve, CommandPS5Controller controller) {
        if (isApril) {
            Optional<Pose3d> aprilPose3d = aprilTagFieldLayout.getTagPose(currentTarget.getFiducialId());
            if(aprilPose3d.isPresent() && eUtil.isIntExistsInArray(currentTarget.getFiducialId(), VisionConstants.reefIDs)){
                return run(() -> swerve.drive(
                    swerve.getTargetSpeeds(
                            controller.getLeftX() * -1 * 0.5,
                            controller.getLeftY() * 0.5,
                            Rotation2d.fromRotations(aprilPose3d.get().toPose2d().getRotation().getRotations()))));
            }
        }
        return run(() -> {
        });
    }

    public Command forwardYawDrive(SwerveSubsystem swerve, CommandPS5Controller controller) {
        double desiredYaw = aprilyaw + VisionConstants.yaw;
        double distance = PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.cameraHeigthtMeters, // Measured with a tape measure, or in CAD.
                VisionConstants.aprilHeightInfo(currentTarget.getFiducialId()), // From 2024 game manual for ID 7
                VisionConstants.cameraPitchInRadians, // Measured with a protractor, or in CAD.
                Units.degreesToRadians(currentTarget.getPitch()));
        if (isApril) {
            return swerve.driveFieldOriented(
                    SwerveInputStream.of(swerve.getSwerveDrive(),
                            () -> MathUtil.clamp(forwardPID.calculate(distance, 0.1), -1, 1) * .25,
                            () -> controller.getLeftX())
                            .withControllerRotationAxis(() -> MathUtil.clamp(turnPID.calculate(desiredYaw, 0), -1, 1)
                                    * 0.25 * -1)
                            .deadband(OperatorConstants.DEADBAND)
                            .scaleTranslation(1));
        }
        return Commands.run(() -> {
        });
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
                    if (eUtil.isIntExistsInArray(target.getFiducialId(), VisionConstants.reefIDs)) {
                        aprilyaw = target.getYaw();
                        isApril = true;
                        currentTarget = target;
                        if (target.getFiducialId() != -1) {
                            isValidApril = true;
                            // SmartDashboard.putNumber("April distance", getAprilDistance());
                            SmartDashboard.putNumber("get current x", target.getBestCameraToTarget().getX());
                            SmartDashboard.putNumber("get cam y", getAprilY());
                            SmartDashboard.putNumber("get z ", getAprilZ());
                        } else {
                            isValidApril = false;
                        }
                        break;
                    }
                }
            } else {
                isApril = false;
                aprilyaw = 0;
                isValidApril = false;
            }
        }
    }
}
