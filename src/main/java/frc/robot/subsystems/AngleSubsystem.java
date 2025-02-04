package frc.robot.subsystems;

import java.lang.reflect.Type;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AngleSubsystem  extends SubsystemBase{
    private SparkMax angleMotor;
    private SparkMaxConfig angleMotorConfig;
    private SparkClosedLoopController angleClosedLoopController;
    private RelativeEncoder encoder;


    public AngleSubsystem() {
        //Device ID will be changed.
        angleMotor = new SparkMax(0, MotorType.kBrushless);

        //Let's initilaze the closed loop control
        angleClosedLoopController = angleMotor.getClosedLoopController();  

        //THIS ENCODER IS %50 FALSE. ONLY USING FOR PROTOTYPING PURPOSES!!!!
        //Could be both the alternate or the absolute encoder. I'm not sure.
        encoder = angleMotor.getAlternateEncoder();

        //Initilazing the config
        angleMotorConfig = new SparkMaxConfig();

        angleMotorConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1)
            //the integer is for the bore encoder. I'M not sure if it's ok to use it or not.
            .countsPerRevolution(8192);  
        
        angleMotorConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
              

        angleMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)

            //PID section
            .p(0.1,ClosedLoopSlot.kSlot1)
            .i(0,ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            //from the example, idk what does it do
            .velocityFF(1.0/5767, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        /*
        angleMotorConfig.softLimit
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(10)
            .
        */
        
        angleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);   
        // Initialize dashboard values
        SmartDashboard.setDefaultNumber("Target Position", 0);
        SmartDashboard.setDefaultNumber("Target Velocity", 0);
        SmartDashboard.setDefaultBoolean("Control Mode", false);
        SmartDashboard.setDefaultBoolean("Reset Encoder", false);
    }


    public void SetAngle(Rotation2d angle){
        //derece, kontrol türü, slot, feedforward!!!! ileride feedforward gerekebilir
        angleClosedLoopController.setReference(
            angle.getDegrees(), 
            ControlType.kPosition,
            ClosedLoopSlot.kSlot1
        ); 
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Actual Position", encoder.getPosition());
        SmartDashboard.putNumber("Actual Velocity", encoder.getVelocity());
    }
}
