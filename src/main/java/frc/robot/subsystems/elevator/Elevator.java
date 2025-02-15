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
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class Elevator extends SubsystemBase {

  private final SparkMax elevMotor;
  private final SparkMax elevFollower;
  private SparkMaxConfig elevConfig;
  private SparkMaxConfig elevFollowerConfig;
  private RelativeEncoder throughbEncoder;
  private SparkClosedLoopController elevController;
  private ElevatorFeedforward feedforward;
  private static double kS = 0;
  private static double kG = 0.7;
  private static double kV = 0.05;
  private static double setpoint;

  public Elevator() {

    elevMotor = new SparkMax(ElevatorConsta
    tants.secondaryMotorPort, MotorType.kBrushless);

    elevConfig = new SparkMaxConfig();
    elevFollowerConfig = new SparkMaxConfig();

    throughbEncoder = elevMotor.getAlternateEncoder();
    elevController = elevMotor.getClosedLoopController();
    feedforward = new ElevatorFeedforward(kS, kG, kV);

    setConfigs();
    applyConfigs();

    
  }

  public void elevSet(double elevHeight) {

    elevController.setReference(
        elevHeight,
        SparkMax.ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        feedforward.calculate(throughbEncoder.getPosition() <= elevHeight ? 1.0 : -1.0),
        ArbFFUnits.kVoltage);

    setpoint = elevHeight;
  }

  public Command elevHoldCommand(){
    return run(()->elevHold());
  }

  public Command setgoal(double goal){
    return run(()->elevSet(goal));
  }

  public Command manualUpCommand(){
    return run(()->elevUp());
  }

  public Command manualDownCommand(){
    return run(()->elevDown());
  }

  public void elevHold() {
    elevSet(throughbEncoder.getPosition());
  }

  public void elevUp() {
    elevMotor.set(0.1);
  }

  public void elevDown() {
    elevMotor.set(-0.1);
  }

  public void elevStop() {
    elevMotor.set(0.0);
  }

  public void elevVoltage(double voltage) {
    elevMotor.setVoltage(voltage);
  }

  /** Method to Apply the configuration to the SPARKs. */
  private void applyConfigs() {
    elevMotor.configure(elevConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevFollower.configure(
        elevFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Set parameters that will apply to all SPARKs. */
  private void setConfigs() {
    elevConfig
        .inverted(true)
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(0.25)
        .closedLoopRampRate(0.15)
        .voltageCompensation(12.0)
        .softLimit
        .reverseSoftLimitEnabled(true)
          .reverseSoftLimit(0)
          .forwardSoftLimitEnabled(true)
          .forwardSoftLimit(4.2);
       

    elevConfig.alternateEncoder.countsPerRevolution(8192).inverted(true);

    elevConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(1.2)
        .i(0)
        .d(1)
        .outputRange(-0.8, 9.5);

    elevConfig
        .signals
        .appliedOutputPeriodMs(5)
        .externalOrAltEncoderPositionAlwaysOn(true)
        .externalOrAltEncoderVelocityAlwaysOn(false)
        .externalOrAltEncoderVelocity(20)
        .externalOrAltEncoderPosition(5)
        .primaryEncoderPositionPeriodMs(500)
        .primaryEncoderVelocityPeriodMs(500);

    elevFollowerConfig.apply(elevConfig).follow(elevMotor);
  }

  @Override
  public void periodic() {
    // Display the applied output of the left and right side onto the dashboard
    SmartDashboard.putNumber("Elev Out", elevMotor.getAppliedOutput());
    SmartDashboard.putNumber("Elev Position", throughbEncoder.getPosition());
    SmartDashboard.putNumber("Elev Setpoint", setpoint);
    SmartDashboard.putNumber("Elev output amps", elevMotor.getOutputCurrent());
    SmartDashboard.putNumber("Elev temp", elevMotor.getMotorTemperature());
    // This method will be called once per scheduler run
  }
}