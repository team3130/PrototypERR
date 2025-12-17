package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;


public class RunOuttake extends Command {
    private final Intake intake;

    public RunOuttake(Intake outtake) {
        this.intake = outtake;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.intake);
    }

    @Override
    public void initialize() {
        intake.runOuttake();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }
}
