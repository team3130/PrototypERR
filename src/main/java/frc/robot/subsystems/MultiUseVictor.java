// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MultiUseVictor extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  private final VictorSPX victor;

  public MultiUseVictor(int CANID) {
    victor = new VictorSPX(CANID);
  }

  public void runAtSpeed(double speed) {
    victor.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    victor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}