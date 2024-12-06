// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Talon1 extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  private final TalonFX talon1;

  public Talon1() {
    talon1 = new TalonFX(Constants.CAN.Talon1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}