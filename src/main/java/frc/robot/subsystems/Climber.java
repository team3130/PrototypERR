// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final TalonFX climberMotor;
  private final DigitalInput limitHome;
  private final DigitalInput limitExtended;
  private final double climbSpeed = 5;

  public Climber() {
    climberMotor = new TalonFX(Constants.CAN.climberMotor, "rio");
    limitHome = new DigitalInput(Constants.IDs.climberLimitHome);
    limitExtended = new DigitalInput(Constants.IDs.climberLimitExtended);
    climberMoter.configFactoryDefault();
  }
  public void stopMoving() {
    climberMotor.set(0);
  }
  public void extend() {
    climberMotor.set(climbSpeed);
  }
  public boolean isExtended() {
    return limitExtended.get();
  }
  public void goHome(){climberMotor.set(-climbSpeed);}
  public boolean isHome() {
    return limitHome.get();
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
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
