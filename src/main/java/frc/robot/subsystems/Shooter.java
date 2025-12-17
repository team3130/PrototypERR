// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



//Make this yourself / CAN ID is Constants.CAN.Falcon / Speed is 1 since it is a flywheel / No brake mode, should be in coast mode

public class Shooter extends SubsystemBase {
  private final TalonFX shooter;
  private double shooterSpeed = 1.0;

  /** Creates a new Shooter. */
  public Shooter() {
    shooter = new TalonFX(Constants.CAN.Falcon);
  }
  public double getShooterSpeed(){
    return shooterSpeed;
  }
  public void setShooterSpeed(double value){
    shooterSpeed = value;
  }
  public void runShooter(){
    shooter.set(shooterSpeed);
  }
  public void reverseShooter() {
    shooter.set(-shooterSpeed);
  }

  public void stopShooter() {
    shooter.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
