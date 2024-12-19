package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
public class Drive extends Command {
    private final Chassis chassis;
    private final PS5Controller controller;

    public Drive(Chassis chassis, PS5Controller ps5Controller){
        this.chassis = chassis;
        this.controller = ps5Controller;

        m_requirements.add(chassis);
    }
    public void initialize() {

    }

    public void execute(){
        chassis.drive(x, y, theta); //uses either driving or targeting inputs for theta
    }

}
