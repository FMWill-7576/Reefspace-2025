package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//FMWill #7576
//AUTHOR : @knkr1

public class ElevatorSubsystem extends SubsystemBase {
    
    private SparkMax mainMotor = new SparkMax(ElevatorConstants.mainMotorPort, MotorType.kBrushless);
    
    private SparkMax secondaryMotor = new SparkMax(ElevatorConstants.secondaryMotorPort, MotorType.kBrushless);


    private SparkMaxConfig mainMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig secondaryMotorConfig = new SparkMaxConfig();

    private RelativeEncoder boreEncoder = mainMotor.getEncoder();
    private SparkClosedLoopController elevatorController;

    private DigitalInput lowLimitSwitch = new DigitalInput(0);

    private double lastestGoal = 0;
    private int currentStateIndex = 0;
    double final_kS = ElevatorConstants.kS;
    double final_kG = ElevatorConstants.kG;
    double final_kV = ElevatorConstants.kV;
    double final_kA = ElevatorConstants.kA;

    double final_kP = ElevatorConstants.kP;
    double final_kI = ElevatorConstants.kI;
    double final_kD = ElevatorConstants.kD;



    public ElevatorSubsystem(){
        setupPreferences();

        //Main Motor
        mainMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ElevatorConstants.smartCurrent);

        mainMotorConfig
            .closedLoop
                .pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, ClosedLoopSlot.kSlot0)
                .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                .outputRange(ElevatorConstants.minOutput, ElevatorConstants.maxOutput);

                //Max motion could be used in the future
                /*
                .maxMotion
                    .maxVelocity(ElevatorConstants.kMaxVelocity,ClosedLoopSlot.kSlot0)
                    .maxAcceleration(ElevatorConstants.kMaxVelocity,ClosedLoopSlot.kSlot0)
                */

        mainMotorConfig.alternateEncoder
            .countsPerRevolution(8192)
            .inverted(false);

        mainMotorConfig.softLimit
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(0)
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(ElevatorConstants.maxPosition);

        
        //Secondary Motor
        secondaryMotorConfig
            .follow(mainMotor);

        mainMotor.configure(mainMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        secondaryMotor.configure(secondaryMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    /**
     * Set elevator voltage to the goal (Basically just set the goal)
     * WARNING! If you setGoal for once, elevator will basicaly apply the same voltage forever.
     * 
     * @param goal    Goal in position measurement of the encoder
     */
    public void setGoal(double goal){
        elevatorController.setReference(
            goal,
            ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot0,
            getLastestFeedforward().calculate(boreEncoder.getPosition())
         );
    }

    /**
     * Set it as the default command, so that it holds the current position.
     * 
     */
    public Command holdPosition() {
        return run(()->setGoal(boreEncoder.getPosition()));
    }

     /**
     * Set the elevator position
     * @param position    The position you want to reach (encoder rotations)
     */
    public Command setElevatorPosition(double position) {
        return run(()->setGoal(position));
    }

    public Command manualElevatorUp(){
        return run(()->mainMotor.set(0.3));
    }

    public Command manualElevatorDown(){
        return run(()->mainMotor.set(-0.3));
    }

    public Command setStateUp() {
        if(currentStateIndex < ElevatorConstants.states.length-1) {
            currentStateIndex += 1;
            return run(()->setGoal(ElevatorConstants.states[currentStateIndex]));
        }
        else {
            //Warn led?
            return run(()->{});
        }
    }

    public Command setStateDown() {
        if(currentStateIndex > 0) {
            currentStateIndex -= 1;
            return run(()->setGoal(ElevatorConstants.states[currentStateIndex]));
        }
        else {
            //Warn led?
            return run(()->{});
        }
    }

    /**
     * Get a BooleanSupplier about if the position at the desired position.
     * @param position    The position
     * @param tolerance   The tolerance
     */
    public BooleanSupplier isAtDesiredPositionAsBooleanSupplier(double position, double tolerance){
        BooleanSupplier sup = () -> MathUtil.isNear(position,boreEncoder.getPosition(),tolerance);
        return sup;
    }

    /**
     * Get a Boolean about if the position at the desired position.
     * @param position    The position
     * @param tolerance   The tolerance
     */
    public Boolean isAtDesiredPositionAsBoolean(double position, double tolerance){
        return MathUtil.isNear(position,boreEncoder.getPosition(),tolerance);
    }

    public ElevatorFeedforward getLastestFeedforward(){

        ElevatorFeedforward feedforward = new ElevatorFeedforward(
            Preferences.getDouble(ElevatorConstants.kS_key, final_kS), 
            Preferences.getDouble(ElevatorConstants.kG_key, final_kG), 
            Preferences.getDouble(ElevatorConstants.kV_key, final_kV), 
            Preferences.getDouble(ElevatorConstants.kA_key, final_kA)
        );

        return feedforward;
    }

    public void SetupLastestPID(){
        mainMotorConfig
            .closedLoop
                .pid(
                    Preferences.getDouble(ElevatorConstants.kP_key, final_kP),
                    Preferences.getDouble(ElevatorConstants.kI_key, final_kI),
                    Preferences.getDouble(ElevatorConstants.kD_key, final_kD),
                    ClosedLoopSlot.kSlot0
                );
        mainMotor.configure(mainMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setupPreferences(){
        Preferences.initDouble(ElevatorConstants.kS_key, final_kS);
        Preferences.initDouble(ElevatorConstants.kG_key, final_kG);
        Preferences.initDouble(ElevatorConstants.kV_key, final_kV);
        Preferences.initDouble(ElevatorConstants.kA_key, final_kA);

        Preferences.initDouble(ElevatorConstants.kP_key, final_kP);
        Preferences.initDouble(ElevatorConstants.kI_key, final_kI);
        Preferences.initDouble(ElevatorConstants.kD_key, final_kD);
    }

    public void log() {
        SmartDashboard.putNumber("Elevator Desired Position", lastestGoal);
        SmartDashboard.putNumber("Elevator Current Position", boreEncoder.getPosition());
        SmartDashboard.putNumber("Elevator Current Motor Output", mainMotor.getAppliedOutput());
    }


    //This is optional
    public void CheckSwitch(){
        //Switch works inverted, it gives 5v when is not getting clicked
        if(!lowLimitSwitch.get()){
            boreEncoder.setPosition(0);
        }
    }

    @Override
    public void periodic() {
        SetupLastestPID();
        log();
        CheckSwitch();
    }
}
