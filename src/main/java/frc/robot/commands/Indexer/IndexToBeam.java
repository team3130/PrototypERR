// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.Command;

//similar to normal indexing but look at my comments in the IsFinished command

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IndexToBeam extends Command {
  /** Creates a new IndexToBeam. */
  public IndexToBeam() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //should just run index normally

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stop index

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if beam is false, return true. How do you do that?
    //Hint: ! will invert from true to false or vice versa
    return false;
  }
}
