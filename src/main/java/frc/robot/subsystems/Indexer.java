// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private final TalonSRX index1;
  private final TalonSRX index2;
  private final DigitalInput indexBeam;

  private double intakeSpeed = 0.3;

  public Indexer() {
    index1 = new TalonSRX(Constants.CAN.Talon3);
    index2 = new TalonSRX(Constants.CAN.Victor4);
    indexBeam = new DigitalInput(0);

    index1.configFactoryDefault();
    index1.setNeutralMode(NeutralMode.Brake);
    index1.setInverted(false);

    index2.configFactoryDefault();
    index2.setNeutralMode(NeutralMode.Brake);
    index2.setInverted(false);
  }

  public void runIndex() {
    index1.set(ControlMode.PercentOutput, intakeSpeed);
    index2.set(ControlMode.PercentOutput, intakeSpeed);
  }
  public void reverseIndex() {
    index1.set(ControlMode.PercentOutput, -intakeSpeed);
    index2.set(ControlMode.PercentOutput, -intakeSpeed);
  }
  public void stopIndex() {
    index1.set(ControlMode.PercentOutput, 0);
    index2.set(ControlMode.PercentOutput, 0);
  }

  public double getIndexSpeed() {return intakeSpeed;}
  public void setIndexSpeed(double speed) {intakeSpeed = speed;}

  public boolean getIndexBeam() {return indexBeam.get();}
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Indexer");

    builder.addDoubleProperty("Index Speed", this::getIndexSpeed, this::setIndexSpeed);
    builder.addBooleanProperty("Index Beam", this::getIndexBeam, null);
  }


  
}
