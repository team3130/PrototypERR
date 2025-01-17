// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.commands.chassis.ResetOdometryForward;
import frc.robot.commands.chassis.RotateTo90;
import frc.robot.commands.chassis.TeleopDrive;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  public final MultiUseTalonSRX multiUseTalon1;
  public final MultiUseTalonSRX multiUseTalon2;
  public final MultiUseTalonSRX multiUseTalon3;
  public final MultiUseVictor multiUseVictor4;
  public final MultiUseTalonSRX multiUseTalon5;
  public final MultiUseTalonFX falcon;


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final PS5Controller driverController = new PS5Controller(0);
  private final XboxController operatorController = new XboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_exampleSubsystem = new ExampleSubsystem();

    chassis = new Chassis();
    multiUseTalon1 = new MultiUseTalonSRX(Constants.CAN.Talon1);
    multiUseTalon2 = new MultiUseTalonSRX(Constants.CAN.Talon2);
    multiUseTalon3 = new MultiUseTalonSRX(Constants.CAN.Talon3);
    multiUseVictor4 = new MultiUseVictor(Constants.CAN.Victor4);
    multiUseTalon5 = new MultiUseTalonSRX(Constants.CAN.Talon5);
    falcon = new MultiUseTalonFX();

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
    new POVButton(driverController, Constants.PS5.POV_N).whileTrue(new ResetOdometryForward(chassis));
    new POVButton(driverController, Constants.PS5.POV_E).whileTrue(new RotateTo90(chassis));

    //new JoystickButton(operatorController, Constants.Xbox.BTN_B).whileTrue(new RunTalon(multiUseTalon1, 0));
    //new JoystickButton(operatorController, Constants.Xbox.BTN_RBUMPER).whileTrue(new RunTalon(multiUseTalon2, 0.35));
    //new JoystickButton(operatorController, Constants.Xbox.BTN_A).whileTrue(new RunTalon(multiUseTalon3, -0.5));
    //new JoystickButton(operatorController, Constants.Xbox.BTN_RBUMPER).whileTrue(new RunVictor(multiUseVictor4));
    //new JoystickButton(operatorController, Constants.Xbox.BTN_X).whileTrue(new RunTalon(multiUseTalon5, 0.4));
    new JoystickButton(driverController, Constants.PS5.BTN_X).whileTrue(new RunTalonFX(falcon, -0.5));
    new JoystickButton(driverController, Constants.PS5.BTN_CIRCLE).whileTrue(new RunTalonFX(falcon, 0.5));
  }

  public void exportShuffleBoardData() {
    if (Constants.debugMode) {
      ShuffleboardTab tab = Shuffleboard.getTab("Subsystem Test");
      tab.add(chassis);
      //tab.add(new RunFalcon(multiUseFalcon, 0.2));

      chassis.exportSwerveModData(Shuffleboard.getTab("Swerve Modules"));
    }
  }

  public void resetOdometryForward() {
    chassis.resetOdometry(new Pose2d(0, 0, new Rotation2d()));
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
