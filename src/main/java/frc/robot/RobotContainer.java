// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Autos;
import frc.robot.commands.RunTalon;
import frc.robot.commands.RunVictor;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.MultiUseTalon;
import frc.robot.subsystems.MultiUseVictor;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final ExampleSubsystem m_exampleSubsystem;
  public final Chassis chassis;

  //Mechanism Motors
  public final MultiUseTalon multiUseTalon1;
  public final MultiUseTalon multiUseTalon2;
  public final MultiUseTalon multiUseTalon3;
  public final MultiUseVictor multiUseVictor4;
  public final MultiUseTalon multiUseTalon5;


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final PS5Controller driverController = new PS5Controller(0);
  private final XboxController operatorController = new XboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_exampleSubsystem = new ExampleSubsystem();

    chassis = new Chassis();
    multiUseTalon1 = new MultiUseTalon(Constants.CAN.Talon1);
    multiUseTalon2 = new MultiUseTalon(Constants.CAN.Talon2);
    multiUseTalon3 = new MultiUseTalon(Constants.CAN.Talon3);
    multiUseVictor4 = new MultiUseVictor(Constants.CAN.Victor4);
    multiUseTalon5 = new MultiUseTalon(Constants.CAN.Talon5);

    // Configure the trigger bindings
    configureBindings();
    exportShuffleBoardData();

    // Set Default commands
    chassis.setDefaultCommand(new TeleopDrive(chassis, driverController));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new JoystickButton(operatorController, Constants.XBox.BTN_A).whileTrue(new RunTalon(multiUseTalon1));
    new JoystickButton(operatorController, Constants.XBox.BTN_B).whileTrue(new RunTalon(multiUseTalon2));
    new JoystickButton(operatorController, Constants.XBox.BTN_X).whileTrue(new RunTalon(multiUseTalon3));
    new JoystickButton(operatorController, Constants.XBox.BTN_Y).whileTrue(new RunVictor(multiUseVictor4));
    new JoystickButton(operatorController, Constants.XBox.BTN_RBUMPER).whileTrue(new RunTalon(multiUseTalon5));
  }

  public void exportShuffleBoardData() {
    if (Constants.debugMode) {
      ShuffleboardTab tab = Shuffleboard.getTab("Subsystem Test");
      tab.add(chassis);

      chassis.exportSwerveModData(Shuffleboard.getTab("Swerve Modules"));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
