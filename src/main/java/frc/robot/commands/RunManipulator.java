// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;

/** An example command that uses an example subsystem. */
public class RunManipulator extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Manipulator talon;
  private double speed = 0;
  /**
   * Creates a new ExampleCommand.
   *
   * @param talon The subsystem used by this command.
   */

  public RunManipulator(Manipulator talon, double speed){
    this.talon = talon;
    this.speed = speed;
    addRequirements(talon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    talon.runAtSpeed(speed);}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {talon.stop();}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
