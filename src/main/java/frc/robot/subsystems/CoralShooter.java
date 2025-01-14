// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;

public class CoralShooter extends SubsystemBase {

  //Since which motor we will use is unclear, ill add them both.
  private SparkMax NeoShooter;

  private SparkMax DummyMotor1;
  private SparkMax DummyMotor2;

  private SparkMaxConfig NeoConfig;

  private SparkMaxConfig DummyConfig1;
  private SparkMaxConfig DummyConfig2;

  SparkClosedLoopController neoClosedLoopController;

  public CoralShooter() {
    NeoShooter = new SparkMax(0, MotorType.kBrushless);

    /*
    //Assuming that we will not use dummies.
    DummyMotor1 = new SparkMax(0, MotorType.kBrushed);
    DummyMotor2 = new SparkMax(0, MotorType.kBrushed);
    */

    NeoConfig
      .inverted(false)
      .idleMode(IdleMode.kCoast);

    NeoConfig.encoder
      .velocityConversionFactor(1.0);

    NeoConfig.closedLoop
      .pid(0, 0, 0)
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .outputRange(0.1, 0.7,ClosedLoopSlot.kSlot2);

    NeoShooter.configure(NeoConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    neoClosedLoopController = NeoShooter.getClosedLoopController();
  }

  public void SetMotorRPM(double rpm){
    neoClosedLoopController.setReference(rpm, ControlType.kVelocity,ClosedLoopSlot.kSlot2);
  }

  //manual 😈
  public void ManualSetMotor(double motorSpeed) {
    NeoShooter.set(motorSpeed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
