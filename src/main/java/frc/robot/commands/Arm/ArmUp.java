package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;


public class ArmUp extends Command {
    private final Arm arm;

    public ArmUp(Arm arm) {
        this.arm = arm;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        arm.armUp();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return !arm.getHighBeam();
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopArm();
    }
}
