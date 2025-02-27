package frc.robot.subsystems.arm;

import java.lang.reflect.Type;

import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;

public class AngleSubsystem  extends SubsystemBase{
    private SparkMax angleMotor;
    private SparkMaxConfig angleMotorConfig;
    private SparkClosedLoopController angleClosedLoopController;
    private RelativeEncoder encoder;
    //private ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.kS,ArmConstants.kG,ArmConstants.kV);
    private double setpoint;



    public AngleSubsystem() {
        //Device ID will be changed.    
        angleMotor = new SparkMax(ArmConstants.kAnglePort, MotorType.kBrushless);

        //Let's initilaze the closed loop control
        angleClosedLoopController = angleMotor.getClosedLoopController();  

        //THIS ENCODER IS %50 FALSE. ONLY USING FOR PROTOTYPING PURPOSES!!!!
        //Could be both the alternate or the absolute encoder. I'm not sure.
        encoder = angleMotor.getAlternateEncoder();

        //Initilazing the config
        angleMotorConfig = new SparkMaxConfig();

        angleMotorConfig.alternateEncoder
            //the integer is for the bore encoder. I'M not sure if it's ok to use it or not.
            .inverted(true);
        
        angleMotorConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
              

        angleMotorConfig.closedLoop 
            .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
            //PID section
            .pid(1,0,0.5,ClosedLoopSlot.kSlot0)
            //from the example, idk what does it do
            .outputRange(-0.3, 0.3);

        
        angleMotorConfig.softLimit
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(0)
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(2);
        
        
        angleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); 
        setupPreferences();
        encoder.setPosition(0);
    }


    public void SetAngle(double angle){
        setpoint = angle;
        angleClosedLoopController.setReference(
            angle, 
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            getArmFeedforward().calculate(encoder.getPosition(),0.5),
            ArbFFUnits.kVoltage
        ); 
    }

    public ArmFeedforward getArmFeedforward() {

       ArmFeedforward a =  new ArmFeedforward(
        Preferences.getDouble("arm_kS", ArmConstants.kS),
        Preferences.getDouble("arm_kG", ArmConstants.kG),
        Preferences.getDouble("arm_kV", ArmConstants.kV)
        );
       return a;
    }

    public void SetAngleRotation(Rotation2d angle){
        //derece, kontrol türü, slot, feedforward!!!! ileride feedforward gerekebilir
        setpoint = angle.getDegrees();
        angleClosedLoopController.setReference(
            angle.getDegrees(), 
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            getArmFeedforward().calculate(angle.getRadians(),0),
            ArbFFUnits.kVoltage
        ); 
    }

    public double getEncoder() {
        return encoder.getPosition();
    }

    public Command setAngleCommand(double goal) {
        return run(()-> SetAngle(goal)).until(()->IsAtDesiredPosition(goal));
    }

    public Command setAngleAsRotationCommand(Rotation2d goal) {
        return run(()-> SetAngleRotation(goal));
    }


    public Command armHold(){
        return run(()->SetAngle(encoder.getPosition()));
    }

    public Command armHoldAsAngle(){
        return run(()->SetAngleRotation(Rotation2d.fromDegrees(encoder.getPosition())));
    }

    public Command armUp(){
        return run(()->angleMotor.set(0.2));
    }
    public Command armStop(){
        return run(()->angleMotor.set(0));
    }

    public Command armDown(){
        return run(()->angleMotor.set(-0.2));
    }

    public Command resetEncoder(){
        return run(()->encoder.setPosition(0));
    }

    public boolean IsAtDesiredPosition(double pos){
        return (MathUtil.isNear(pos, getEncoder(), 0.1));
    }

    public Command setArmHome(){
        return setAngleCommand(ArmConstants.homeSetpoint);
    }


    public Command setArmSafe(){
        return run(()->SetAngle(2)).until(()->IsAtDesiredPosition(2));
    }

    public void setupPreferences() {
        Preferences.initDouble("arm_kS", ArmConstants.kS);
        Preferences.initDouble("arm_kV", ArmConstants.kV);
        Preferences.initDouble("arm_kG", ArmConstants.kG);
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", encoder.getPosition());
        SmartDashboard.putNumber("Arm Output", angleMotor.getAppliedOutput());
        SmartDashboard.putNumber("Setpoint Arm", setpoint);
    }
}
