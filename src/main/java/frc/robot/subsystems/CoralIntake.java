// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
  private final WPI_TalonSRX coralIntake;
  private final int coralIntakeVoltage = 5;
  private final double coralIntakeSpeed = .6;
  /** Creates a new ExampleSubsystem. */
  public CoralIntake() {
    coralIntake = new WPI_TalonSRX(Constants.CAN.coralIntake);

    coralIntake.configFactoryDefault();
    coralIntake.setNeutralMode(NeutralMode.Coast);
    coralIntake.enableVoltageCompensation(true);
    coralIntake.configVoltageCompSaturation(coralIntakeVoltage);
    coralIntake.setInverted(false);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void intakeCoral (){
    coralIntake.set(coralIntakeSpeed);
  }

  public void outakeCoral(){
    coralIntake.set(-coralIntakeSpeed);
  }

  public void coralStop(){
    coralIntake.stopMotor();
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
