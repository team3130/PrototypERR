// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private final WPI_TalonSRX elevator;
  private final XboxController xboxController;
  private double elevatorSpeed = .9;
  private double elevatorVoltage = 10;

  public Elevator() {
  xboxController = new XboxController(Constants.Xbox.AXS_RJOYSTICK_Y);

  elevator = new WPI_TalonSRX(Constants.CAN.elevatorMotor);

  elevator.configFactoryDefault();
  elevator.configVoltageCompSaturation(elevatorVoltage);
  elevator.enableVoltageCompensation(true);
  elevator.setNeutralMode(NeutralMode.Brake);
  elevator.setInverted(false);
  }

  public void lift(){
    elevator.set(elevatorSpeed);
  }

  public void lower(){
    elevator.set(-elevatorSpeed);
  }

  public void stop(){
    elevator.stopMotor();
  }

  public double getElevatorVelocity(){
    double elevatorVelocity = -xboxController.getRawAxis(Constants.Xbox.AXS_RJOYSTICK_Y);
    return elevatorVelocity * Math.abs(elevatorVelocity);
  }

  public void proportionalLift(double elevatorVelocity){
    elevator.set(elevatorVelocity);
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
