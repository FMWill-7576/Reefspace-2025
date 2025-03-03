// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;
import java.util.Optional;

import org.ejml.sparse.csc.misc.ImplCommonOpsWithSemiRing_DSCC;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ShooterSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import swervelib.SwerveDrive;

public class LedSubsystem extends SubsystemBase {

  private AddressableLED insideLed;
  private AddressableLEDBuffer insideLed_buffer;

  private AddressableLEDBufferView insideLed_buffer_left;
  private AddressableLEDBufferView insideLed_buffer_right;
  private AddressableLEDBufferView insideLed_buffer_middle;

  private int Insidelenght = 74 + 48 + 2;
  private final Distance InsideSpace = Meters.of(1 / Insidelenght);


  private Elevator s_elevator;
  private VisionSubsystem s_vision;
  private ShooterSubsystem s_shooter;



  public LedSubsystem(Elevator elev, VisionSubsystem vision,ShooterSubsystem shooter) {
    s_elevator = elev;
    s_vision=vision;
    s_shooter = shooter;
    // Insider led
    insideLed = new AddressableLED(0);
    insideLed_buffer = new AddressableLEDBuffer(Insidelenght);
    insideLed.setLength(insideLed_buffer.getLength());

    insideLed_buffer_left = insideLed_buffer.createView(0, 47);
    insideLed_buffer_middle = insideLed_buffer.createView(48, 73);
    insideLed_buffer_right = insideLed_buffer.createView(74, 74 + 48).reversed();

    insideLed.start();


    setLedBlank();
  }

  public void setAllianceColor() {
    LEDPattern blueTeam = LEDPattern.solid(Color.kDarkBlue).breathe(Seconds.of(5)).atBrightness(Percent.of(50));
    LEDPattern redTeam = LEDPattern.solid(Color.kDarkRed).breathe(Seconds.of(5)).atBrightness(Percent.of(50));

    Optional<Alliance> ally = DriverStation.getAlliance();
    if(ally.get()==Alliance.Red){
      redTeam.applyTo(insideLed_buffer);
    }else if(ally.get()==Alliance.Blue) {
      blueTeam.applyTo(insideLed_buffer);
    }else {
      LEDPattern pattern = LEDPattern.solid(Color.kAqua).blink(Seconds.of(3));
      pattern.applyTo(insideLed_buffer);
      insideLed.setData(insideLed_buffer);
    }
    insideLed.setData(insideLed_buffer);
  }

  public void solidAllianceColor() {
    LEDPattern blueTeam = LEDPattern.solid(Color.kDarkBlue).atBrightness(Percent.of(50));
    LEDPattern redTeam = LEDPattern.solid(Color.kDarkRed).atBrightness(Percent.of(50));

    Optional<Alliance> ally = DriverStation.getAlliance();
    if(ally.get()==Alliance.Red){
      redTeam.applyTo(insideLed_buffer);
    }else {
      blueTeam.applyTo(insideLed_buffer);
    }
    insideLed.setData(insideLed_buffer);
  }

  public void setLedBlank() {
    LEDPattern black = LEDPattern.solid(Color.kBlack);
    black.applyTo(insideLed_buffer);
    insideLed.setData(insideLed_buffer);
  }

  public void setCustomColor(Color color){
    LEDPattern pattern = LEDPattern.solid(color);
    pattern.applyTo(insideLed_buffer);
    insideLed.setData(insideLed_buffer);
  }

  public void greenBlink() {
    LEDPattern base = LEDPattern.solid(Color.kGreen).blink(Seconds.of(1));
    base.applyTo(insideLed_buffer);
    insideLed.setData(insideLed_buffer);
  }

  public void greenFlash () {
    LEDPattern base = LEDPattern.solid(Color.kGreen).breathe(Seconds.of(0.25));
    base.applyTo(insideLed_buffer);
    insideLed.setData(insideLed_buffer);
  }

  public void CoralLed() {
    LEDPattern pattern = LEDPattern.solid(Color.kDarkOrange).blink(Seconds.of(0.25));
    pattern.applyTo(insideLed_buffer);
    insideLed.setData(insideLed_buffer);
  }

  public void allianceFlash() {
    LEDPattern blueTeam = LEDPattern.solid(Color.kDarkBlue).blink(Seconds.of(0.5));
    LEDPattern redTeam = LEDPattern.solid(Color.kDarkRed).blink(Seconds.of(0.5));

    Optional<Alliance> ally = DriverStation.getAlliance();
    if(ally.get()==Alliance.Red){
      redTeam.applyTo(insideLed_buffer);
    }else {
      blueTeam.applyTo(insideLed_buffer);
    }
    insideLed.setData(insideLed_buffer);
  }

  public Command LedCommand() {
    return run(()->{
      if(MathUtil.isNear(0.29, s_vision.getAprilY(), 0.02)){
        greenFlash();
      }else if(s_shooter.IsCoral()){
        allianceFlash();
      }else{
        solidAllianceColor();
      }
    });
  }

  @Override
  public void periodic() {
    if(DriverStation.isDisabled()) {
      setAllianceColor();
    }
  }
}
