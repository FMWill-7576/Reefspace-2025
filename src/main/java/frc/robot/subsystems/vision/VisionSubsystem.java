package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class VisionSubsystem extends SubsystemBase {

    String limelightName = "";

    PIDController rotational_PID = new PIDController(0.05, 0, 0);
    PIDController translationalY_PID = new PIDController(0.05, 0, 0);

    public VisionSubsystem() {
        rotational_PID.setTolerance(VisionConstants.horizontalTolerance);
        translationalY_PID.setTolerance(VisionConstants.areaTolerance);
    }

    public boolean IsAprilTag() {
        return LimelightHelpers.getTV(limelightName);
    }

    public double getHorizontalOffset() {
        return LimelightHelpers.getTX(limelightName);
    }

    public double getArea() {
        return LimelightHelpers.getTX(limelightName);
    }

    public double getDistance() {
        return LimelightHelpers.getTY(limelightName);
    }

    public int getAprilTagID() {

        return (int) LimelightHelpers.getFiducialID(limelightName);
    }

    public Command driveLeftAllign(SwerveSubsystem drivebase, CommandPS5Controller controller) {
        // Make it parallel
        double left_rotationalAxis = MathUtil.clamp(
                rotational_PID.calculate(
                        getHorizontalOffset(),
                        VisionConstants.leftHorizontalOffset),
                -0.5, 0.5) * 1;

        // Move forward
        double left_translationY_Axis = MathUtil.clamp(
                rotational_PID.calculate(
                        getArea(),
                        VisionConstants.leftArea),
                -0.5, 0.5) * 1;

        return run(() -> drivebase.driveFieldOriented(
            SwerveInputStream.of(drivebase.getSwerveDrive(),
                () -> left_translationY_Axis,
                () -> 0)
                .withControllerRotationAxis(()->left_rotationalAxis)
                .allianceRelativeControl(true)
                )
            );
    }

    public boolean CanShootVision(){
        if(MathUtil.isNear(VisionConstants.leftArea, getArea(), VisionConstants.areaTolerance) && MathUtil.isNear(VisionConstants.leftHorizontalOffset, getArea(), VisionConstants.horizontalTolerance)){
            return true;
        }else if(MathUtil.isNear(VisionConstants.rightArea, getArea(), VisionConstants.areaTolerance) && MathUtil.isNear(VisionConstants.rightHorizontalOffset, getArea(), VisionConstants.horizontalTolerance)){
            return true;
        }
        else {
            return false;
        }
    }

    public Command driveRightAllign(SwerveSubsystem drivebase, CommandPS5Controller controller) {
        // Make it parallel
        double right_rotationalAxis = MathUtil.clamp(
                rotational_PID.calculate(
                        getHorizontalOffset(),
                        VisionConstants.rightHorizontalOffset),
                -0.5, 0.5) * 1;

        // Move forward
        double right_translationY_Axis = MathUtil.clamp(
                rotational_PID.calculate(
                        getArea(),
                        VisionConstants.rightArea),
                -0.5, 0.5) * 1;

        return run(() -> drivebase.driveFieldOriented(
            SwerveInputStream.of(drivebase.getSwerveDrive(),
                () -> right_translationY_Axis,
                () -> 0)
                .withControllerRotationAxis(()->right_rotationalAxis)
                .allianceRelativeControl(true)
                )
            );
    }

    public Command logBothAllign() {

        // Make it parallel
        double left_rotationalAxis = MathUtil.clamp(
                rotational_PID.calculate(
                        getHorizontalOffset(),
                        VisionConstants.leftHorizontalOffset),
                -0.5, 0.5) * 1;

        // Move forward
        double left_translationY_Axis = MathUtil.clamp(
                rotational_PID.calculate(
                        getArea(),
                        VisionConstants.leftArea),
                -0.5, 0.5) * 1;

        // Make it parallel
        double right_rotationalAxis = MathUtil.clamp(
                rotational_PID.calculate(
                        getHorizontalOffset(),
                        VisionConstants.rightHorizontalOffset),
                -0.5, 0.5) * 1;

        // Move forward
        double right_translationY_Axis = MathUtil.clamp(
                rotational_PID.calculate(
                        getArea(),
                        VisionConstants.rightArea),
                -0.5, 0.5) * 1;

        return run(() -> {
            SmartDashboard.putNumber("right calculated translation", right_translationY_Axis);
            SmartDashboard.putNumber("left calculated translation", left_translationY_Axis);
            SmartDashboard.putNumber("left calculated rotation", left_rotationalAxis);
            SmartDashboard.putNumber("right calculated rotation", right_rotationalAxis);
        });

    }

    public void log() {
        SmartDashboard.putBoolean("IsApril", IsAprilTag());
        SmartDashboard.putNumber("April Area", getArea());
        SmartDashboard.putNumber("April Horizontal", getHorizontalOffset());
    }

    @Override
    public void periodic() {
        log();
    }
}
