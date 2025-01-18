// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralManipulator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final WPI_TalonSRX coralManipulator;
  private double manipulatorSpeed = .4;
  private double manipulatorVoltage = 5;

  public CoralManipulator() {
    coralManipulator = new WPI_TalonSRX(Constants.CAN.coralManipulator);

    coralManipulator.configFactoryDefault();
    coralManipulator.setNeutralMode(NeutralMode.Brake);
    coralManipulator.enableVoltageCompensation(true);
    coralManipulator.configVoltageCompSaturation(manipulatorVoltage);
    coralManipulator.setInverted(false);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public void manipSpin(){
    coralManipulator.set(manipulatorSpeed);
  }

  public void manipSpout(){
    coralManipulator.set(-manipulatorSpeed);
  }

  public void manipStop(){
    coralManipulator.stopMotor();
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
