// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    private final TalonFX arm;
    private double armSpeed = 0.75;
    private final DigitalInput lowBeam;
    private final DigitalInput highBeam;

    public Arm() {
        arm = new TalonFX(Constants.CAN.Arm);
        arm.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake));
        arm.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true).withSupplyCurrentLimit(40));
        lowBeam = new DigitalInput(LowBeam);
        highBeam = new DigitalInput(HighBeam);
    }

    public void armUp() {
        arm.set(armSpeed);
    }
    public void armDown() {
        arm.set(-armSpeed);
    }
    public void stopArm() {
        arm.set(0);
    }

    public double getArmSpeed() {return armSpeed;}
    public void setArmSpeed(double value) {armSpeed = value;}

    public double getPosition() {return arm.getPosition().getValueAsDouble();}

    public boolean getLowBeam() {return lowBeam.get();}
    public boolean getHighBeam() {return highBeam.get();}

    /**
     * Initializes the data we send on shuffleboard
     * Calls the default init sendable for Subsystem Bases
     * @param builder sendable builder
     */
    public void initSendable(SendableBuilder builder) {
        if (Constants.debugMode) {
            builder.setSmartDashboardType("Arm");

            builder.addDoubleProperty("Arm Speed", this::getArmSpeed, this::setArmSpeed);
            builder.addDoubleProperty("Position", this::getPosition, null);
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
