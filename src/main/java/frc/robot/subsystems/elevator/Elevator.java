package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  private final SparkMax mainMotor = new SparkMax(ElevatorConstants.mainMotorPort, MotorType.kBrushless);
  private SparkMax secondaryMotor = new SparkMax(ElevatorConstants.secondaryMotorPort, MotorType.kBrushless);
  private SparkMaxConfig mainConfig = new SparkMaxConfig();
  private SparkMaxConfig secondaryConfig = new SparkMaxConfig();
  private RelativeEncoder boreEncoder = mainMotor.getAlternateEncoder();
  private SparkClosedLoopController controller = mainMotor.getClosedLoopController();
  private static double setpoint;

  public static double kS = 0;
  public static double kG = 0.7;
  public static double kV = 0;
  public static double kA = 0.05;

  //For state change
  public int currentIndex = 0;

  public Elevator() {
    mainConfig
        .inverted(true)
        .smartCurrentLimit(ElevatorConstants.smartCurrent)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(0.25)
        .closedLoopRampRate(0.15)
        .softLimit
          .reverseSoftLimitEnabled(true)
          .reverseSoftLimit(0)
          .forwardSoftLimitEnabled(true)
          .forwardSoftLimit(4.5);

    mainConfig.alternateEncoder
        .countsPerRevolution(8192)
        .inverted(true);

    mainConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD)
        .outputRange(ElevatorConstants.minOutput, ElevatorConstants.maxOutput);

    secondaryConfig
        .apply(mainConfig)
        .follow(mainMotor);
    mainMotor.configure(mainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    secondaryMotor.configure(secondaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setupPreferences();
  }

  public void setPosition(double elevHeight) {

    controller.setReference(
        elevHeight,
        SparkMax.ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        getLastestFeedforward().calculate(boreEncoder.getPosition() <= elevHeight ? 1.0 : -1.0),
        ArbFFUnits.kVoltage);

    setpoint = elevHeight;
  }

  public ElevatorFeedforward getLastestFeedforward() {
    ElevatorFeedforward newFeed = new ElevatorFeedforward(
      Preferences.getDouble(ElevatorConstants.kS_key, kS), 
      Preferences.getDouble(ElevatorConstants.kG_key, kG), 
      Preferences.getDouble(ElevatorConstants.kV_key, kV)
    );
    return newFeed;
  }

  //State Holding
  public void GoCurrentState(){
    setPosition(ElevatorConstants.states[currentIndex]);
  }

  public void StateUp() {
    if(currentIndex<ElevatorConstants.states.length){
      currentIndex+=1;
    }
    GoCurrentState();
  }
  public Command StateUpCommand(){
    return run(()->StateUp());
  }

  public void StateDown() {
    if(currentIndex>0){
      currentIndex-=1;
    }
    GoCurrentState();
  }
  public Command StateDownCommand(){
    return run(()->StateDown());
  }


  //misc
  public Command elevHoldCommand() {
    return run(() -> elevHold());
  }

  public Command setgoal(double goal) {
    return run(() -> setPosition(goal));
  }

  public Command manualUpCommand() {
    return run(() -> elevUp());
  }

  public Command manualDownCommand() {
    return run(() -> elevDown());
  }

  public void elevHold() {
    setPosition(boreEncoder.getPosition());
  }

  public void elevUp() {
    mainMotor.set(0.1);
  }

  public void elevDown() {
    mainMotor.set(-0.1);
  }

  public void elevStop() {
    mainMotor.set(0.0);
  }

  public void elevVoltage(double voltage) {
    mainMotor.setVoltage(voltage);
  }

  public void setupPreferences(){
      Preferences.initDouble(ElevatorConstants.kS_key, kS);
      Preferences.initDouble(ElevatorConstants.kG_key, kG);
      Preferences.initDouble(ElevatorConstants.kV_key, kV);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elev Out", mainMotor.getAppliedOutput());
    SmartDashboard.putNumber("Elev Position", boreEncoder.getPosition());
    SmartDashboard.putNumber("Elev Setpoint", setpoint);
    SmartDashboard.putNumber("Elev output amps", mainMotor.getOutputCurrent());
    SmartDashboard.putNumber("Elev temp", mainMotor.getMotorTemperature());
  }
}