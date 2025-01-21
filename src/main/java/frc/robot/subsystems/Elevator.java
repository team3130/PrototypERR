// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final DigitalInput limitSwitch;
  private final CANcoder encoder;

  private double home = 0;
  private double minPosition = 0;
  private double L1 = 0;
  private double L2 = 0;
  private double L3 = 0;
  private double L4 = 0;
  private double maxPosition = 0;

  private final MotionMagicVoltage voltRequest0;
  private Slot0Configs slot0Configs;
  private double slot0kG = 0;
  private double slot0kP = 0;
  private double slot0kI = 0;
  private double slot0kD = 0;

  private final MotionMagicVoltage voltRequest1;
  private final Slot1Configs slot1Configs;
  private double slot1kG = 0;
  private double slot1kP = 0;
  private double slot1kI = 0;
  private double slot1kD = 0;

  private boolean zeroed = false;
  public Elevator() {
    leftMotor = new TalonFX(Constants.CAN.ElevatorLeft);
    rightMotor = new TalonFX(Constants.CAN.ElevatorRight);
    limitSwitch = new DigitalInput(Constants.IDs.ElevatorLimitSwitch);
    encoder = new CANcoder(Constants.IDs.ElevatorEncoder);

    encoder.getConfigurator().apply(new CANcoderConfiguration());

    leftMotor.getConfigurator().apply(new TalonFXConfiguration());
    rightMotor.getConfigurator().apply(new TalonFXConfiguration());

    leftMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
    rightMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    leftMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(6));
    rightMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(6));

    voltRequest0 = new MotionMagicVoltage(0);
    slot0Configs = new Slot0Configs().withGravityType(GravityTypeValue.Elevator_Static);
    slot0Configs.kG = slot0kG;
    slot0Configs.kP = slot0kP;
    slot0Configs.kI = slot0kI;
    slot0Configs.kD = slot0kD;

    voltRequest1 = new MotionMagicVoltage(0);
    slot1Configs = new Slot1Configs().withGravityType(GravityTypeValue.Elevator_Static);
    slot1Configs.kG = slot1kG;
    slot1Configs.kP = slot1kP;
    slot1Configs.kI = slot1kI;
    slot1Configs.kD = slot1kD;

    leftMotor.getConfigurator().apply(slot0Configs);
    rightMotor.getConfigurator().apply(slot1Configs);
  }

  public void stop() {
    leftMotor.setControl(voltRequest0.withPosition(0));
    rightMotor.setControl(voltRequest1.withPosition(0));
  }

  public void goDown() {
    leftMotor.set(-0.2);
    rightMotor.set(-0.2);
  }

  public void goUp() {
    leftMotor.set(0.2);
    rightMotor.set(0.2);
  }

  public void goToSetpoint(double setpoint) {
    leftMotor.setControl(voltRequest0.withPosition(setpoint));
    rightMotor.setControl(voltRequest1.withPosition(setpoint));
  }

  public void goToHome() {
    goDown();
    if(brokeLimitSwitch()) {
      setPosition(0);
      setZeroed(true);
    }
  }

  public void goToL1() {
    goToSetpoint(L1);
  }
  public void goToL2() {
    goToSetpoint(L2);
  }
  public void goToL3() {
    goToSetpoint(L3);
  }
  public void goToL4() {
    goToSetpoint(L4);
  }

  public double getPosition() {return encoder.getPosition().getValueAsDouble();}
  public void setPosition(double value) {encoder.setPosition(value);}

  public boolean brokeLimitSwitch() {return limitSwitch.get();}

  public boolean isZeroed() {return zeroed;}
  public void setZeroed(boolean value) {zeroed = value;}

  public double getHome() {return home;}
  public double getMinPosition() {return minPosition;}
  public double getL1() {return L1;}
  public double getL2() {return L2;}
  public double getL3() {return L3;}
  public double getL4() {return L4;}
  public double getMaxPosition() {return maxPosition;}
  public void setHome(double value) {home = value;}
  public void setMinPosition(double value) {minPosition = value;}
  public void setL1(double value) {L1 = value;}
  public void setL2(double value) {L2 = value;}
  public void setL3(double value) {L3 = value;}
  public void setL4(double value) {L4 = value;}
  public void setMaxPosition(double value) {maxPosition = value;}

  public double getSlot0kG() {return slot0kG;}
  public double getSlot0kP() {return slot0kP;}
  public double getSlot0kI() {return slot0kI;}
  public double getSlot0kD() {return slot0kD;}
  public void setSlot0kG(double value) {slot0kG = value;}
  public void setSlot0kP(double value) {slot0kG = value;}
  public void setSlot0kI(double value) {slot0kG = value;}
  public void setSlot0kD(double value) {slot0kG = value;}

  public double getSlot1kG() {return slot1kG;}
  public double getSlot1kP() {return slot1kP;}
  public double getSlot1kI() {return slot1kI;}
  public double getSlot1kD() {return slot1kD;}
  public void setSlot1kG(double value) {slot1kG = value;}
  public void setSlot1kP(double value) {slot1kP = value;}
  public void setSlot1kI(double value) {slot1kI = value;}
  public void setSlot1kD(double value) {slot1kD = value;}


  /**
   * Initializes the data we send on shuffleboard
   * Calls the default init sendable for Subsystem Bases
   * @param builder sendable builder
   */
  public void initSendable(SendableBuilder builder) {
    if (Constants.debugMode) {
      builder.setSmartDashboardType("Elevator");

      builder.addDoubleProperty("Position", this::getPosition, null);
      builder.addDoubleProperty("Min Position", this::getMinPosition, this::setMinPosition);
      builder.addDoubleProperty("Home", this::getHome, this::setHome);
      builder.addDoubleProperty("L1", this::getL1, this::setL1);
      builder.addDoubleProperty("L2", this::getL2, this::setL2);
      builder.addDoubleProperty("L3", this::getL3, this::setL3);
      builder.addDoubleProperty("L4", this::getL4, this::setL4);
      builder.addDoubleProperty("Max Position", this::getMaxPosition, this::setMaxPosition);

      builder.addDoubleProperty("Slot 0 kG", this::getSlot0kG, this::setSlot0kG);
      builder.addDoubleProperty("Slot 0 kP", this::getSlot0kP, this::setSlot0kP);
      builder.addDoubleProperty("Slot 0 kI", this::getSlot0kI, this::setSlot0kI);
      builder.addDoubleProperty("Slot 0 kD", this::getSlot0kD, this::setSlot0kD);
      builder.addDoubleProperty("Slot 1 kG", this::getSlot1kG, this::setSlot1kG);
      builder.addDoubleProperty("Slot 1 kP", this::getSlot1kP, this::setSlot1kP);
      builder.addDoubleProperty("Slot 1 kI", this::getSlot1kI, this::setSlot1kI);
      builder.addDoubleProperty("Slot 1 kD", this::getSlot1kD, this::setSlot1kD);

      builder.addBooleanProperty("Broke Limit", this::brokeLimitSwitch, null);
      builder.addBooleanProperty("Is Zeroed", this::isZeroed, this::setZeroed);
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
