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

    PIDController rotational_PID = new PIDController(0.005, 0, 0);
    PIDController translational_PID = new PIDController(0.01, 0, 0);

    public VisionSubsystem() {
        rotational_PID.setTolerance(VisionConstants.horizontalTolerance);
        translational_PID.setTolerance(VisionConstants.areaTolerance);
    }


    public boolean IsAprilTag() {
        return LimelightHelpers.getTV(limelightName);
    }

    public double getHorizontalOffset() {
        return LimelightHelpers.getTX(limelightName);
    }
    

    /*
     * Use for distance
     */
    public double getArea() {
        return LimelightHelpers.getTX(limelightName);
    }

    public int getAprilTagID() {

        return (int) LimelightHelpers.getFiducialID(limelightName);
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

    public double get_right_rotation(){
        if(getArea()!=0 && getHorizontalOffset()!=0){
            double val = MathUtil.clamp(
                rotational_PID.calculate(
                        getHorizontalOffset(),
                        VisionConstants.rightHorizontalOffset),
                -1, 1) * 1;
            return val;
        }
        return 0;
    }

    public double get_left_rotation(){
        if(getArea()!=0 && getHorizontalOffset()!=0){
            double val = MathUtil.clamp(
            rotational_PID.calculate(
                    getHorizontalOffset(),
                    VisionConstants.leftHorizontalOffset),
            -1, 1) * 1;
        return val;
        }
        return 0;
    }

    public double get_left_translation(){
        if(getArea()!=0 && getHorizontalOffset()!=0){
            double val = MathUtil.clamp(
            translational_PID.calculate(
                    getArea(),
                    VisionConstants.leftArea),
            -1, 1) * 1;
        return val;
        }
        return 0;
    }



    public double get_right_translation(){
        if(getArea()!=0 && getHorizontalOffset()!=0){
            double val = MathUtil.clamp(
            translational_PID.calculate(
                    getArea(),
                    VisionConstants.rightArea),
            -1, 1) * 1;
        return val;
        }
        return 0;
    }

    public Command driveRightAllign(SwerveSubsystem drivebase, CommandPS5Controller controller) {
        return drivebase.drive(
            SwerveInputStream.of(drivebase.getSwerveDrive(),
                () -> get_right_translation(),
                () -> 0)
                .withControllerRotationAxis(()->get_right_rotation())
                .allianceRelativeControl(false)
                .robotRelative(true)
        );
    }   
    public Command driveLeftAllign(SwerveSubsystem drivebase, CommandPS5Controller controller) {
        return drivebase.drive(
            SwerveInputStream.of(drivebase.getSwerveDrive(),
                () -> get_left_translation(),
                () -> 0)
                .withControllerRotationAxis(()->get_left_rotation())
                .allianceRelativeControl(false)
                .robotRelative(true)
            );
    }   

    public void logBothAllign() {

        // Make it parallel
        double left_rotationalAxis = MathUtil.clamp(
                rotational_PID.calculate(
                        getHorizontalOffset(),
                        VisionConstants.leftHorizontalOffset),
                -1, 1) * 1;

        // Move forward
        double left_translationY_Axis = MathUtil.clamp(
                rotational_PID.calculate(
                        getArea(),
                        VisionConstants.leftArea),
                -1, 1) * 1;

        // Make it parallel
        double right_rotationalAxis = MathUtil.clamp(
                rotational_PID.calculate(
                        getHorizontalOffset(),
                        VisionConstants.rightHorizontalOffset),
                -1, 1) * 1;

        // Move forward
        double right_translationY_Axis = MathUtil.clamp(
                rotational_PID.calculate(
                        getArea(),
                        VisionConstants.rightArea),
                -1, 1) * 1;

        SmartDashboard.putNumber("left rot pid", left_rotationalAxis);
        SmartDashboard.putNumber("left translation pid", left_translationY_Axis);

    }

    public void log() {
        SmartDashboard.putBoolean("IsApril", IsAprilTag());
        SmartDashboard.putNumber("April Area", getArea());
        SmartDashboard.putNumber("April Horizontal", getHorizontalOffset());
    }

    @Override
    public void periodic() {
        log();
        logBothAllign();
    }
}
