// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDeeznut extends SubsystemBase {
AddressableLED led;
AddressableLEDBuffer ledBuffer;
public int LEDport = 9;
public int LEDlength = 60;

  public LEDeeznut() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    led = new AddressableLED(LEDport);
    // Create an LED pattern that sets the entire strip to solid red
    LEDPattern brown = LEDPattern.solid(Color.kBrown);

// Apply the LED pattern to the data buffer
    brown.applyTo(ledBuffer);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    ledBuffer = new AddressableLEDBuffer(LEDlength);
    led.setLength(ledBuffer.getLength());

    // Set the data
    led.setData(ledBuffer);
    led.start();
  }

  public void redlight(){
    LEDPattern red = LEDPattern.solid(Color.kRed);
    red.applyTo(ledBuffer);
    led.setData(ledBuffer);
    led.start();
  }
  public void rainbow(){
    LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    rainbow.applyTo(ledBuffer);
    led.setData(ledBuffer);
    led.start();
  }
  public void gradient(){
    LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kAqua);
    gradient.applyTo(ledBuffer);
    led.setData(ledBuffer);
    led.start();
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
