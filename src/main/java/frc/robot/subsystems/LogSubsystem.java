// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LogSubsystem extends SubsystemBase {
  
  private CANcoder frontRightCan = new CANcoder(1);
  private CANcoder frontLeftCan = new CANcoder(1);
  private CANcoder backRightCan = new CANcoder(1);
  private CANcoder backLeftCan = new CANcoder(1);

  public LogSubsystem() {}

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("front left cancoder", frontLeftCan.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("front right cancoder", frontRightCan.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("back left cancoder", backLeftCan.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("back right cancoder", backRightCan.getPosition().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
