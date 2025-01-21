// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//written in phoenix 5 since TalonSRX doesn't exist in phoenix 6
public class CoralIntake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final TalonSRX intake;
  private final TalonSRX pivot;
  private final DigitalInput limit;
  private double intakeSpeed = 0.5;
  private double pivotSpeed = 0.5;
  public CoralIntake() {
    intake = new TalonSRX(Constants.CAN.CoralIntake);
    pivot = new TalonSRX(Constants.CAN.CoralIntakePivot);
    limit = new DigitalInput(Constants.IDs.IntakeLimitSwitch);

    pivot.configFactoryDefault();
    pivot.setInverted(false);

    intake.configFactoryDefault();
    intake.setInverted(false);
  }

  public void stopIntake() {
    intake.set(ControlMode.PercentOutput, 0);
  }

  public void runIntake() {
    intake.set(ControlMode.PercentOutput, intakeSpeed);
  }

  public void runOuttake() {
    intake.set(ControlMode.PercentOutput, -intakeSpeed);
  }

  public void stopPivot() {
    pivot.set(ControlMode.PercentOutput, 0);
  }

  public void pivotDown() {
    pivot.set(ControlMode.PercentOutput, -pivotSpeed);
  }

  public void pivotUp() {
    pivot.set(ControlMode.PercentOutput, pivotSpeed);
  }

  public double getIntakeSpeed() {return intakeSpeed;}
  public void setIntakeSpeed(double value) {intakeSpeed = value;}

  public double getPivotSpeed() {return pivotSpeed;}
  public void setPivotSpeed(double value) {pivotSpeed = value;}

  public boolean brokeLimit() {return limit.get();}

  /**
   * Initializes the data we send on shuffleboard
   * Calls the default init sendable for Subsystem Bases
   * @param builder sendable builder
   */
  public void initSendable(SendableBuilder builder) {
    if (Constants.debugMode) {
      builder.setSmartDashboardType("Coral Intake");

      builder.addDoubleProperty("Intake Speed", this::getIntakeSpeed, this::setIntakeSpeed);
      builder.addDoubleProperty("Pivot Speed", this::getPivotSpeed, this::setPivotSpeed);

      builder.addBooleanProperty("Intake Limit", this::brokeLimit, null);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
