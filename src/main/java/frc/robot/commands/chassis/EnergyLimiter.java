package frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Time;

public class EnergyLimiter {
    public Translation2d joystick;
    public double rotation;
    public Translation2d currentVel;
    public Translation2d ghostStick;
    public Time nowtime;
    public Time prevtime;

    EnergyLimiter(Translation2d joystick) {
        this.joystick = joystick;
        this.currentVel = 
    }
}
