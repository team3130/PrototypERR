// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonSRX intake;
  private double intakeSpeed = 0.3;         //always keep this a small value when testing at first, preferably below 0.5 (50%)
  public Intake() {
    intake = new TalonSRX(Constants.CAN.Talon2);  //Constants.CAN.Talon2 is just a number represented in Constants file.
    intake.configFactoryDefault();
    intake.setInverted(false);       //always keep this false when testing at first. change if needed
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

  //Make a getter and setter for intake speed below




  public void InitSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Intake");

    //This command sends stuff to SmartDashboard (elastic, glass, etc.)
    //Replace "getter" and "setter" with the name of getter and setter functions you define
    //Uncomment line when ready

    //builder.addDoubleProperty("Intake Speed", this::getter, this::setter);

    /*
     * Example:
     * builder.addDoubleProperty("Intake Speed", this::getIntakeSpeed, this::setIntakeSpeed)
     */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}