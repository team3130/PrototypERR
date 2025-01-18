// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final WPI_TalonSRX climber;
  private double climberVoltage = 12;
  private double climbSpeed = .9;

  public Climber() {
    climber = new WPI_TalonSRX(Constants.CAN.climber);

    climber.configFactoryDefault();
    climber.enableVoltageCompensation(true);
    climber.setNeutralMode(NeutralMode.Brake);
    climber.configVoltageCompSaturation(climberVoltage);
    climber.setInverted(false);

  }

  public void climb(){
    climber.set(climbSpeed);
  }
  public void unClimb(){
    climber.set(-climbSpeed);
  }
  public void stopClimb(){
    climber.stopMotor();
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
