package frc.robot.subsystems.elevator;

import java.util.function.BooleanSupplier;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.AngleSubsystem;

public class Elevator extends SubsystemBase {

  private final SparkMax mainMotor = new SparkMax(ElevatorConstants.mainMotorPort, MotorType.kBrushless);
  private SparkMax secondaryMotor = new SparkMax(ElevatorConstants.secondaryMotorPort, MotorType.kBrushless);
  private SparkMaxConfig mainConfig = new SparkMaxConfig();
  private SparkMaxConfig secondaryConfig = new SparkMaxConfig();
  private RelativeEncoder boreEncoder = mainMotor.getAlternateEncoder();
  private SparkClosedLoopController controller = mainMotor.getClosedLoopController();
  private static double setpoint;

  public static double kS = ElevatorConstants.kS;
  public static double kG = ElevatorConstants.kG;
  public static double kV = ElevatorConstants.kV;

  //For state change
  public static int currentIndex = 0;

  public Elevator() {
    mainConfig
        .inverted(true)
        .smartCurrentLimit(ElevatorConstants.smartCurrent)
        .idleMode(IdleMode.kCoast)
        .openLoopRampRate(0.25)
        .closedLoopRampRate(0.15)
        .softLimit
          .reverseSoftLimitEnabled(true)
          .reverseSoftLimit(0)
          .forwardSoftLimitEnabled(true)
          .forwardSoftLimit(ElevatorConstants.maxPosition);

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
    mainMotor.configure(mainConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    secondaryMotor.configure(secondaryConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    setupPreferences();
  }

  public void setPosition(double elevHeight) {
    setpoint = elevHeight;
    controller.setReference(
        elevHeight,
        SparkMax.ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        getLastestFeedforward().calculate(boreEncoder.getPosition() <= elevHeight ? 1.0 : -1.0),
        ArbFFUnits.kVoltage);
  }

  public ElevatorFeedforward getLastestFeedforward() {
    ElevatorFeedforward newFeed = new ElevatorFeedforward(
      Preferences.getDouble(ElevatorConstants.kS_key, kS), 
      Preferences.getDouble(ElevatorConstants.kG_key, kG), 
      Preferences.getDouble(ElevatorConstants.kV_key, kV)
    );
    return newFeed;
  }

  public boolean IsAtDesiredHeight(double height){
    return MathUtil.isNear(height, boreEncoder.getPosition(), 0.3);
  }

  public double getCurrentSetpoint() {
    return setpoint;
  }

  //State Holding
  public Command StateUpCommand(){
    if(currentIndex<ElevatorConstants.states.length-1){
      currentIndex = currentIndex+1;
    }
    return this.run(()->setPosition(ElevatorConstants.states[currentIndex])).until(()->IsAtDesiredHeight(ElevatorConstants.states[currentIndex]));
  }

  public Command StateDownCommand(){
    if(currentIndex>0){
      currentIndex = currentIndex - 1;
    }
    return this.run(()->setPosition(ElevatorConstants.states[currentIndex])).until(()->IsAtDesiredHeight(ElevatorConstants.states[currentIndex]));
  }


  public int getCurrentState() {
    double pos = boreEncoder.getPosition();
    if(MathUtil.isNear(ElevatorConstants.states[0], pos, 0.5)){
      return 1;
    }else if(MathUtil.isNear(ElevatorConstants.states[1], pos, 0.5)){
      return 2;
    }else if(MathUtil.isNear(ElevatorConstants.states[2], pos, 0.5)){
      return 3;
    }else if(MathUtil.isNear(ElevatorConstants.states[3], pos, 0.5)){
      return 4;
    }
    return 100;
  }


  //misc
  public double getPosition() {
    return boreEncoder.getPosition();
  }
  public Command elevHoldCommand() {
    return run(() -> elevHold());
  }

  public Command setgoal(double goal) {
    return run(() -> setPosition(goal)).until(()->IsAtDesiredHeight(goal));
  }

  public Command manualUpCommand() {
    return run(() -> setpoint+=0.02);
  }

  public Command manualDownCommand() {
    return run(() -> setpoint-=0.02);
  }
  public Command manualStopCommand() {
    return run(() -> setpoint+=0.02);
  }

  public void elevHold() {
    setPosition(boreEncoder.getPosition());
  }

  public Command holdAtSetpoint() {
    return run(()->setPosition(setpoint));
  }

  public Command setSetpoint(double set){
    return run(()->setpoint=0);
  }

  public void elevUp() {
    setpoint+=0.04;
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

  public int getCurrentIndex() {
    return currentIndex;
  }
  public void setCurrentIndex(int n) {
    currentIndex = n;
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
    SmartDashboard.putNumber("current index", currentIndex);
    SmartDashboard.putNumber("Current state", getCurrentState());
  }
}