// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonSRX intake;
  private double intakeSpeed = 0.6;
  public Intake() {
    intake = new TalonSRX(0);
    intake.configFactoryDefault();
    intake.setInverted(true);
  }

  public void runIntake() {
    intake.set(ControlMode.PercentOutput, intakeSpeed);
  }
  public void reverseIntake() {
    intake.set(ControlMode.PercentOutput, -intakeSpeed);
  }
  public void stopIntake() {
    intake.set(ControlMode.PercentOutput, 0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
