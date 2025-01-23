// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase {
  private final TalonFX talon;
  /** Creates a new ExampleSubsystem. */

  public Manipulator() {
    talon = new TalonFX(Constants.CAN.Falcon, "rio");
  }

  public void runAtSpeed(double speed) {
    talon.set(speed);
  }
  public void stop() {
    talon.set(0);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
