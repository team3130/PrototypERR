// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private final TalonFX climberMotor;
  private final DigitalInput limitHome;
  private final DigitalInput limitExtended;
  private final double climbSpeed = 5;

  public Climber() {
    climberMotor = new TalonFX(Constants.CAN.climberMotor, "rio");
    limitHome = new DigitalInput(Constants.IDs.climberLimitHome);
    limitExtended = new DigitalInput(Constants.IDs.climberLimitExtended);
    climberMotor.getConfigurator().apply(new TalonFXConfiguration());
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
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
}
