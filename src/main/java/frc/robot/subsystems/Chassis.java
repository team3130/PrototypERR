// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.Navx;

public class Chassis extends SubsystemBase {
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator odometry;
    private final SwerveModule[] modules;
    private final Navx navx = Navx.GetInstance();

    public Chassis() {
        this (new Pose2d(), new Rotation2d());
    }

    public Chassis(Pose2d pose2d, Rotation2d rotation2d) {
        kinematics = new SwerveDriveKinematics(Constants.Swerve.moduleTranslations);

        modules = new SwerveModule[4];
        modules[0] = new SwerveModule(Constants.SwerveModules.one);
        modules[1] = new SwerveModule(Constants.SwerveModules.two);
        modules[2] = new SwerveModule(Constants.SwerveModules.three);
        modules[3] = new SwerveModule(Constants.SwerveModules.four);

        odometry = new SwerveDrivePoseEstimator(kinematics, rotation2d, getPositions(), pose2d);
    }
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(int i = 0; i <= 3; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }


    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
