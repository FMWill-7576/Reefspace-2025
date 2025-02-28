// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import swervelib.SwerveDrive;

public class LedSubsystem extends SubsystemBase {

  private AddressableLED insideLed;
  private AddressableLEDBuffer insideLed_buffer;

  private AddressableLEDBufferView insideLed_buffer_left;
  private AddressableLEDBufferView insideLed_buffer_right;
  private AddressableLEDBufferView insideLed_buffer_middle;

  private AddressableLED elevLed;
  private AddressableLEDBuffer elevLed_buffer;
  private int elevLedLenght = 200;

  private int Insidelenght = 74+48+2;
  private Elevator elevSub;

  private final Distance InsideSpace = Meters.of(1/Insidelenght);

  public LedSubsystem(Elevator s_elevator) {
    elevSub = s_elevator;
    
    //Insider led
     insideLed = new AddressableLED(0);
     insideLed_buffer = new AddressableLEDBuffer(Insidelenght);
     insideLed.setLength(insideLed_buffer.getLength());

     elevSub = s_elevator;
  
     insideLed_buffer_left = insideLed_buffer.createView(0, 47);
     insideLed_buffer_middle = insideLed_buffer.createView(48, 73);
     insideLed_buffer_right = insideLed_buffer.createView(74, 74+48).reversed();
    
     insideLed.start();
  }

  public void setAllianceColorCommand() {
    LEDPattern blueTeam = LEDPattern.gradient(GradientType.kContinuous, Color.kBlue,Color.kDodgerBlue);
    LEDPattern redTeam = LEDPattern.gradient(GradientType.kContinuous, Color.kRed,Color.kOrangeRed);
    //sinsideLed_buffer_middle.
    //Commands.either(()->, null, DriverStation.getAlliance() == DriverStation.Alliance.Red()
  }


  @Override
  public void periodic() {
    insideLed.setData(insideLed_buffer);
  }
}
