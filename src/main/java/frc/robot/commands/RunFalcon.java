// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.MultiUseFalcon;

/** An example command that uses an example subsystem. */
public class RunFalcon extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final MultiUseFalcon falcon;
  private double speed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param falcon The subsystem used by this command.
   */
  public RunFalcon(MultiUseFalcon falcon, double speed) {
    this.falcon = falcon;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(falcon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    falcon.runAtSpeed(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    falcon.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
