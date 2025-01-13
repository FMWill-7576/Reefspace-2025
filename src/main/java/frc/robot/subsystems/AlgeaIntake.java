// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.AlternateEncoderConfig.Type;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class AlgeaIntake extends SubsystemBase {

  //Intake için
  private SparkMax intakeMotor; 
  SparkMaxConfig intakeConfig;

  //Intake açısı için, %100 neo brushless
  private SparkMax angleMotor; 
  SparkMaxConfig angleConfig;

  SparkClosedLoopController angle_Controller;

  ArmFeedforward armFeedforward;

  private double kS = 0.0;
  private double kV = 0.0;
  private double kG = 0.0;
  private double kA = 0.0;

  public AlgeaIntake() {

    // Intake motor ID will be changed soon.
    //Eğer 775 motoru ise (Brushed)
    //intakeMotor = new SparkMax(9,MotorType.kBrushed)
    //Eğer 775 motoru ise (Brushless)
    intakeMotor = new SparkMax(9,MotorType.kBrushless);
    intakeConfig = new SparkMaxConfig();


    intakeConfig
      .inverted(false) // sets motor direction
      .idleMode(IdleMode.kCoast)
      .openLoopRampRate(0.5)
      .closedLoopRampRate(0.5);

    intakeMotor.configure(intakeConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    angleMotor = new SparkMax(8, MotorType.kBrushless);
    angleConfig = new SparkMaxConfig();
    angleConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .closedLoopRampRate(0.5)
      .openLoopRampRate(0.5);

    angleConfig.alternateEncoder
      .inverted(false);

    angleConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
      .pid(0, 0, 0)
      //May change soon, it will set the speed
      .outputRange(0.1, 0.5, ClosedLoopSlot.kSlot1);
      
    angleMotor.configure(angleConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    angle_Controller =  angleMotor.getClosedLoopController();

    armFeedforward = new ArmFeedforward(kS,kG,kV,kA);
  }

  public void SetAlgeaIntake(double speed) {
    intakeMotor.set(0.5);
  }

  public void SetAlgeaAngle(Rotation2d rotation) {
    angle_Controller.setReference(
      rotation.getDegrees(), 
      ControlType.kPosition,ClosedLoopSlot.kSlot1,armFeedforward.calculate(rotation.getRadians(), 1,1)
      );
  }

  public void SetManualAlgeaAngle(double power) {
    angleMotor.set(power);
  }


  public Command exampleMethodCommand() {
    return runOnce(
        () -> {
        });
  }
  public boolean exampleCondition() {
    return false;
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
