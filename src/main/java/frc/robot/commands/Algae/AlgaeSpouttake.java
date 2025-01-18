// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Algae;

import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


/** An example command that uses an example subsystem. */
public class AlgaeSpouttake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AlgaeIntake algaeIntake;

  public AlgaeSpouttake(AlgaeIntake AlgaeIntake) {
    algaeIntake = AlgaeIntake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeIntake.reverseAlgaeIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeIntake.stopAlgaeIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
