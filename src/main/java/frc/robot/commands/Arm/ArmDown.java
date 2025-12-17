package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;


public class ArmDown extends Command {
    private final Arm arm;

    public ArmDown(Arm arm) {
        this.arm = arm;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        arm.armDown();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return !arm.getLowBeam();
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopArm();
    }
}
