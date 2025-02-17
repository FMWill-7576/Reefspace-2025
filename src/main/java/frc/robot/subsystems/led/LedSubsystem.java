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
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import swervelib.SwerveDrive;

public class LedSubsystem extends SubsystemBase {

  private AddressableLED s_led;
  private AddressableLEDBuffer s_buffer;

  private AddressableLEDBufferView s_buffer_left;
  private AddressableLEDBufferView s_buffer_right;
  private AddressableLEDBufferView s_buffer_middle;

  private final LEDPattern rainbow = LEDPattern.rainbow(255, 128);
  private static final Distance ledspace = Meters.of(1/120.0);
  private final LEDPattern scroll = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), ledspace);

  private int lenght = 74+48+2;
  private Elevator elevSub;

  public LedSubsystem(Elevator s_elevator) {
    //PWM port
     s_led = new AddressableLED(0);
     //Lenght
     s_buffer = new AddressableLEDBuffer(lenght);
     s_led.setLength(s_buffer.getLength());

     elevSub = s_elevator;

     //Experimental, may use later.
  
     s_buffer_left = s_buffer.createView(0, 47);
     s_buffer_middle = s_buffer.createView(48, 73);
     s_buffer_right = s_buffer.createView(74, 74+48).reversed();
     s_led.setLength(s_buffer.getLength());
    

    LEDPattern baseOrange = LEDPattern.solid(Color.kCrimson);
    LEDPattern baseAqua = LEDPattern.solid(Color.kSeaGreen);
    LEDPattern baseBlue = LEDPattern.solid(Color.kDarkSeaGreen);

    LEDPattern zero = LEDPattern.solid(Color.kBlack);

    zero.applyTo(s_buffer);

    //baseOrange.applyTo(s_buffer_left);
    //baseAqua.applyTo(s_buffer_right);
    baseBlue.applyTo(s_buffer_middle);
    s_led.setData(s_buffer);
    s_led.start();
  }


  @Override
  public void periodic() {
    LEDPattern pattern = LEDPattern.progressMaskLayer(()-> elevSub.getPosition()/ElevatorConstants.maxPosition);
    pattern.applyTo(s_buffer_left);
    pattern.applyTo(s_buffer_right);
    s_led.setData(s_buffer);
  }

  public Command runPattern(LEDPattern pattern){
    return run(()->{
      pattern.applyTo(s_buffer);
    });
  }

  public Command Blue(){
    return(run(()->{
      LEDPattern base = LEDPattern.solid(Color.kBlue);
      //LEDPattern pattern = base.breathe(Seconds.of(2));

      base.applyTo(s_buffer);
    }));
  }
}
