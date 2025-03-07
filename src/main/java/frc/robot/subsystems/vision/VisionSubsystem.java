package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class VisionSubsystem extends SubsystemBase {

    String limelightName = "";

    PIDController rotational_PID = new PIDController(0.05, 0, 0);
    PIDController translationalX_PID = new PIDController(0.05, 0, 0);
    PIDController translationalY_PID = new PIDController(0.05, 0, 0);

    public VisionSubsystem() {

    }

    public boolean IsAprilTag(){
        return LimelightHelpers.getTV(limelightName);
    }

    public double getHorizontalOffset() {
        return LimelightHelpers.getTX(limelightName);
    }

    public double getArea() {
        return LimelightHelpers.getTX(limelightName);
    }

    public int getAprilTagID(){
    
        return (int)LimelightHelpers.getFiducialID(limelightName);
    }

    public Command driveLeftAllign(SwerveSubsystem drivebase, CommandPS5Controller controller) {
        double rotationalAxis = MathUtil.clamp(rotational_PID.calculate(getHori, getArea()), -0.5, 0.5)
        return run(()->{

        });
    }

    public Command logBothAllign(){

        //Make it parallel
        double left_rotationalAxis = MathUtil.clamp(
            rotational_PID.calculate(
                getHorizontalOffset(), 
                VisionConstants.leftHorizontalOffset),
             -0.5, 0.5);
        
        //Move left-right
        double left_translationX_axis = MathUtil.clamp(
            rotational_PID.calculate(
                getHorizontalOffset(), 
                VisionConstants.leftArea),
            -0.5, 0.5);
        
        //Move forward
        double left_translationY_Axis = MathUtil.clamp(
            rotational_PID.calculate(
                getArea(), 
                VisionConstants.leftArea),
            -0.5, 0.5);

        

        
    }


    public void log(){
        SmartDashboard.putBoolean("IsApril", IsAprilTag());
        SmartDashboard.putNumber("April Area", getArea());
        SmartDashboard.putNumber("April Horizontal", getHorizontalOffset());
    }

    @Override
    public void periodic() {
        log();
    }
}
