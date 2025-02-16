package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorConstants;

/*
 * Author: @knkr1
 * FMWILL
 * #7576
 */

public class VisionSubsystem extends SubsystemBase {
    PhotonCamera camera;
    Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonPoseEstimator photonPoseEstimator;
    double final_kP = VisionConstants.kP;
    double final_kI = VisionConstants.kI;
    double final_kD = VisionConstants.kD;

    public VisionSubsystem() {
        camera = new PhotonCamera("7576vision");
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //facing forward, 0.5meter forward of center, 0.5 meter up
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCam);
    }



    public PIDController getLastestPID() {
        PIDController s = new PIDController(Preferences.getDouble(VisionConstants.kP_string, final_kP), Preferences.getDouble(VisionConstants.kP_string, final_kI), Preferences.getDouble(VisionConstants.kP_string, final_kD));
        return s;
    }

    public double getAprilTagYawAsRadian(int id){
        Pose3d pose = aprilTagFieldLayout.getTagPose(id).get();
        return pose.getRotation().getAngle();
    }

    public PhotonTrackedTarget bestTarget(){
        List<PhotonPipelineResult> result = camera.getAllUnreadResults();
        return result.get(0).getBestTarget();
    } 

    @Override
    public void periodic(){

    }
}
