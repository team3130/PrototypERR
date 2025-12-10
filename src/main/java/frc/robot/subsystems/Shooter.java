// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final TalonFX shooter;
  
  private double shooterSpeed = 1;
  private double reverseShooterSpeed = -0.3;

  /** Creates a new Shooter. */
  public Shooter() {
    shooter = new TalonFX(Constants.CAN.Falcon);

    shooter.getConfigurator().apply(new TalonFXConfiguration());
    shooter.getConfigurator().apply(new MotorOutputConfigs()
    .withNeutralMode(NeutralModeValue.Coast)
    .withInverted(InvertedValue.Clockwise_Positive));
  }

  public void shoot() {
    shooter.set(shooterSpeed);
  }
  public void reverseShoot() {
    shooter.set(reverseShooterSpeed);
  }
  public void stopShooter() {
    shooter.set(0);
  }

  public double getShooterSpeed() {return shooterSpeed;}
  public void setShooterSpeed(double speed) {shooterSpeed = speed;}

  public double getReverseShooterSpeed() {return reverseShooterSpeed;}
  public void setReverseShooterSpeed(double speed) {reverseShooterSpeed = speed;}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Indexer");

    builder.addDoubleProperty("Shooter Speed", this::getShooterSpeed, this::setShooterSpeed);
    builder.addDoubleProperty("Reverse Shooter Speed", this::getReverseShooterSpeed, this::setReverseShooterSpeed);
  }
}
