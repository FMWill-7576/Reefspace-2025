// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;

public class LedSubsystem extends SubsystemBase {

  private AddressableLED s_led;
  private AddressableLEDBuffer s_buffer;

  private AddressableLEDBufferView s_buffer_left;
  private AddressableLEDBufferView s_buffer_right;

  public LedSubsystem() {
    //PWM port
     s_led = new AddressableLED(0);
     //Lenght
     s_buffer = new AddressableLEDBuffer(40);

     //Experimental, may use later.
     /*
     s_buffer_left = s_buffer.createView(0, 59);
     s_buffer_right = s_buffer.createView(60, 120).reversed();
     s_led.setLength(s_buffer.getLength());
      */

     s_led.setData(s_buffer);
     s_led.start();
  }


  @Override
  public void periodic() {
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
