package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;


public class RunOuttake extends Command {
    private final Intake outtake;

    public RunOuttake(Intake outtake) {
        this.outtake = outtake;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.outtake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        outtake.runOuttake();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        outtake.stopIntake();
    }
}
