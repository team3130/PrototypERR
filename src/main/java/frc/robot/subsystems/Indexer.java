// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private final TalonSRX index1;
  private final TalonSRX index2;
  private double intakeSpeed = 0.6;
  public Indexer() {
    index1 = new TalonSRX(0);
    index2 = new TalonSRX(0);
    index1.configFactoryDefault();
    index1.setInverted(true);
    index2.configFactoryDefault();
    index2.setInverted(true);
  }

  public void runIndex() {
    index1.set(ControlMode.PercentOutput, intakeSpeed);
    index2.set(ControlMode.PercentOutput, intakeSpeed);
  }
  public void reverseIndex() {
    index1.set(ControlMode.PercentOutput, -intakeSpeed);
    index2.set(ControlMode.PercentOutput, -intakeSpeed);
  }
  public void stopIntake() {
    index1.set(ControlMode.PercentOutput, 0);
    index2.set(ControlMode.PercentOutput, 0);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  



  
}
