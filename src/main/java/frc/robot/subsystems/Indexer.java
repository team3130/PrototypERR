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

  //uncomment line below whenever you read this
  //private final DigitalInput indexBeam;

  private double intakeSpeed = 0.6;
  public Indexer() {
    index1 = new TalonSRX(Constants.CAN.Talon3);      //Constants.CAN.Talon3 is just a number represented in Constants file.
    index2 = new TalonSRX(Constants.CAN.Victor4);     //Constants.CAN.Victor4 is just a number represented in Constants file.
    //See if you can initialize the intake beam yourself. The ID should be 0 for now

    index1.configFactoryDefault();
    index1.setInverted(true);
    index1.setNeutralMode(NeutralMode.Brake);         //when the motor is set to 0, enable the brake. do this for index2 as well

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
  
  //getter and setter for intake speed



  //getter for beam. indexBeam.get() will give you the value you want to return



  public void InitSendable(SendableBuilder builder){
    // Do this yourself using Intake subsystem as inspiration

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
