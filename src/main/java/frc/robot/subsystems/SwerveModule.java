package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import frc.robot.Constants;


// Swerve module that reflects the actual swerve modules
public class SwerveModule implements Sendable {
    private final TalonFX steerMotor; // the steering motor
    private final TalonFX driveMotor; // the driving motor
    private final CANcoder absoluteEncoder; // the can encoder attached to the shaft
    private final PIDController turningPidController; // PID controller for steering
    private final double absoluteEncoderOffset; // the absolute encoder offset from where 0 is to where it thinks it is
    private final int side; // the side that the bot is on
    final VoltageOut steerMotorVoltRequest = new VoltageOut(0);
    final VoltageOut driveMotorVoltRequest = new VoltageOut(0);
    final TalonFXConfiguration steerTalonConfig = new TalonFXConfiguration();
    final TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration();

    final VelocityDutyCycle driveVeloRequest = new VelocityDutyCycle(0);
    Slot0Configs slot0DriveConfigs;
    private double drive_kS = 0;
    private double drive_kV = 0.124;
    private double drive_kA = 0;
    private double drive_kP = 0;
    private double drive_kI = 0;
    private double drive_kD = 0;

    final MotionMagicDutyCycle steerPositionRequest = new MotionMagicDutyCycle(0);
    Slot0Configs slot0SteerConfigs;
    private double steer_kP = 10;
    private double steer_kI = 0;
    private double steer_kD = 0.1;


    //private double steeringVoltage = 4d;
    //private double drivingVoltage = 10d;

    /**
     * Initializes a swerve module and its motors.
     * Initializes the steering PID controller.
     * @param side is reflective in {@link Constants}
     */
    public SwerveModule(int side) {
        steerMotor = new TalonFX(Constants.Swerve.turningID[side]);
        driveMotor = new TalonFX(Constants.Swerve.spinningID[side]);

        absoluteEncoder = new CANcoder(Constants.Swerve.CANCoders[side]);
        turningPidController = new PIDController(Constants.Swerve.turning_kP_Swerve[side], Constants.Swerve.turning_kI_Swerve[side], Constants.Swerve.turning_kD_Swerve[side]);

        steerTalonConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive); //set config to clockwise
        steerTalonConfig.ClosedLoopGeneral.withContinuousWrap(true);
        steerMotor.getConfigurator().apply(steerTalonConfig); // config factory default
        steerMotor.setNeutralMode(NeutralModeValue.Brake); // Brake mode
        //steerMotor.setInverted(true);

        driveTalonConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive); //set config to counter-clockwise
        driveMotor.getConfigurator().apply(driveTalonConfig);
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        //driveMotor.setInverted(false);

        turningPidController.enableContinuousInput(-Math.PI, Math.PI); // wrap for circles
        turningPidController.setTolerance(0.0025, 0.05); // at position tolerance

        absoluteEncoderOffset = Constants.SwerveEncoderOffsets.kCANCoderOffsets[side];
        this.side =side;

        slot0DriveConfigs = new Slot0Configs();
        slot0DriveConfigs.kS = drive_kS;
        slot0DriveConfigs.kV = drive_kV;
        slot0DriveConfigs.kA = drive_kA;
        slot0DriveConfigs.kP = drive_kP;
        slot0DriveConfigs.kI = drive_kI;
        slot0DriveConfigs.kD = drive_kD;
        driveMotor.getConfigurator().apply(slot0DriveConfigs);

        slot0SteerConfigs = new Slot0Configs();
        slot0SteerConfigs.kP = steer_kP;
        slot0SteerConfigs.kI = steer_kI;
        slot0SteerConfigs.kD = steer_kD;
        steerMotor.getConfigurator().apply(slot0SteerConfigs);


        driveMotor.getConfigurator().apply((new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).
                withSupplyCurrentLimit(40).withSupplyCurrentLowerTime(0)), 0.01);

        steerMotor.getConfigurator().apply((new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true).
                withSupplyCurrentLimit(20).withSupplyCurrentLowerTime(0)), 0.01);

        driveMotor.getConfigurator().apply((new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true).
                withStatorCurrentLimit(40).withSupplyCurrentLowerTime(0)), 0.01);

        steerMotor.getConfigurator().apply((new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true).
                withStatorCurrentLimit(20).withSupplyCurrentLowerTime(0)), 0.01);


        resetEncoders();

        String name = this.getClass().getSimpleName();
        name = name.substring(name.lastIndexOf('.') + 1);
        name += " " + side;
        SendableRegistry.addLW(this, name, name);
    }

    // returns the amount of distance the drive motor has travelled in meters
    public double getDrivePosition() {
        if (side == 0 || side == 1) {
            return driveMotor.getPosition().getValueAsDouble() * Constants.SwerveConversions.frontDriveRotToMeters;
        } else {
            return driveMotor.getPosition().getValueAsDouble() * Constants.SwerveConversions.backDriveRotToMeters;
        }
    }

    // returns the position of the steering motor radians
    public Rotation2d getTurningPosition() {
        // return steerMotor.getPosition().getValue() * Constants.Conversions.SteerRotToRads;
        if (side == 0 || side == 1) {
            return new Rotation2d(steerMotor.getPosition().getValueAsDouble() * Constants.SwerveConversions.frontSteerRotToRads);
        } else {
            return new Rotation2d(steerMotor.getPosition().getValueAsDouble() * Constants.SwerveConversions.backSteerRotToRads);
        }
    }

    public double getTurningPositionRadians() {
        if (side == 0 || side == 1) {
            return steerMotor.getPosition().getValueAsDouble() * Constants.SwerveConversions.frontSteerRotToRads;
        } else {
            return steerMotor.getPosition().getValueAsDouble() * Constants.SwerveConversions.backSteerRotToRads;
        }
    }

    // gets the velocity of the drive motor in m/s
    public double getDriveVelocity() {
        if (side == 0 || side == 1) {
            return driveMotor.getVelocity().getValueAsDouble() * Constants.SwerveConversions.frontDriveRotToMeters;
        } else {
            return driveMotor.getVelocity().getValueAsDouble() * Constants.SwerveConversions.backDriveRotToMeters;
        }
    }

    // gets the speed at which the steering motor turns in radians per second
    public double getTurningVelocity() {
        if (side == 0 || side == 1) {
            return steerMotor.getVelocity().getValueAsDouble() * Constants.SwerveConversions.frontSteerRotToRads * 10d;
        } else {
            return steerMotor.getVelocity().getValueAsDouble() * Constants.SwerveConversions.backSteerRotToRads * 10d;
        }
    }

    // gets the position of the steering wheel according to the absolute encoders
    public double getAbsoluteEncoderRots() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getAbsoluteEncoderRotation() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public void updatePValue(double p) {
        turningPidController.setP(p);
    }
    public void updateDValue(double d) {
        turningPidController.setD(d);
    }

    /**
     * Resets the relative encoders according the absolute encoder involving the offset
     */
    public void resetEncoders() {
        if (side == 0 || side == 1) {
            steerMotor.setPosition((getAbsoluteEncoderRots() - absoluteEncoderOffset) * Constants.SwerveConversions.frontSteerGearRatio);
        } else {
            steerMotor.setPosition((getAbsoluteEncoderRots() - absoluteEncoderOffset) * Constants.SwerveConversions.backSteerGearRatio);
        }
        //steerMotor.setPosition((getAbsoluteEncoderRad() - absoluteEncoderOffset) / Constants.Conversions.SteerRotToRads);
    }

    /**
     * Whether the wheels are zeroed or not
     * @return custom at set-point logic for the PID controller
     */
    public boolean wheelsZeroed() {
        Rotation2d pos = getTurningPosition();
        return (pos.getDegrees() > 355 || pos.getDegrees() < 5) && getTurningVelocity() < 0.05;
    }

    /**
     * @return the current swerve module state
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getTurningPosition());
    }

    //Method runs just with speed
    public void basicRun(double speed) {
        steerMotor.setControl(steerPositionRequest.withPosition(0));
        driveMotor.setControl(driveVeloRequest.withVelocity(speed));
    }

    // Default stop method to stop the motors
    public void stop() {
        steerMotor.setControl(steerMotorVoltRequest.withOutput(0));
        driveMotor.setControl(driveMotorVoltRequest.withOutput(0));
    }

    /**
     * Set the desired swerve module state
     * @param state the state to set the swerve modules to
     */
    public void setTeleopDesiredState(SwerveModuleState state) {
        // dead-band
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // max turn is 90 degrees optimization
        state.optimize(getState().angle);
        // percent output of the drive motor that the swerve controller wants you to go to by the physical max speed the bot can travel
        // m_driveMotor.setControl(driveMotorVoltRequest.withOutput(12d* (state.speedMetersPerSecond / Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond)));
        driveMotor.setVoltage(Constants.Swerve.maxDriveVoltage * (state.speedMetersPerSecond / Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond));
        // set the steering motor based off the output of the PID controller
        steerMotor.setVoltage(Constants.Swerve.maxSteerVoltage * turningPidController.calculate(Math.IEEEremainder(getTurningPositionRadians(), Math.PI * 2), state.angle.getRadians()));
    }

    public void setAutonDesiredState(SwerveModuleState state) {
        // dead-band
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // max turn is 90 degrees optimization
        state.optimize(getState().angle);
        // percent output of the drive motor that the swerve controller wants you to go to by the physical max speed the bot can travel
        // m_driveMotor.setControl(driveMotorVoltRequest.withOutput(12d* (state.speedMetersPerSecond / Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond)));
        driveMotor.setVoltage((10d* (state.speedMetersPerSecond / Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond)));
        // set the steering motor based off the output of the PID controller
        steerMotor.setVoltage(4d * turningPidController.calculate(Math.IEEEremainder(getTurningPositionRadians(), Math.PI * 2), state.angle.getRadians()));
    }

    public void setTeleopDesiredStateVelo(SwerveModuleState state) {
        // dead-band
        if(Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        updateDrivePIDValues(); //mostly for testing
        updateSteerPIDValues(); //mostly for testing

        //max turn is 90 degree optimization
        state.optimize(getState().angle);
        //set velocity to speed from state for drive motor
        driveMotor.setControl(driveVeloRequest.withVelocity(state.speedMetersPerSecond));
        //set the steering motor based off the output of the PID controller
        steerMotor.setControl(steerPositionRequest.withPosition(state.angle.getRadians()));
    }

    public void driveAtVelocity( double velocity) {
        updateDrivePIDValues();
        steerMotor.setControl(steerPositionRequest.withPosition(0));
        driveMotor.setControl(driveVeloRequest.withVelocity(velocity));
    }

    /**
     * Turns the motors to an angle
     * @param setpoint in radians
     */
    public void turnToAngle(double setpoint) {
        //steerMotor.setVoltage(12d * turningPidController.calculate(Math.IEEEremainder(getTurningPositionRadians(), Math.PI * 2), setpoint));
        steerMotor.setControl(steerPositionRequest.withPosition(setpoint));
    }


    /**
     * Get the position of the swerve module
     * @return gets with turning position and velocity
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getTurningPosition());
    }

    /**
     * Whether the pid controller is at the set point.
     * @return whether pid is done
     */
    public boolean PIDisDone() {
        return turningPidController.atSetpoint();
    }

    public void setPValue(double newP) {
        turningPidController.setP(newP);
    }
    public void setDValue(double newD) {
        turningPidController.setD(newD);
    }
    public void setIValue(double newI) {
        turningPidController.setI(newI);
    }

    public double getPValue() {
        return turningPidController.getP();
    }
    public double getIValue() {
        return turningPidController.getI();
    }
    public double getDValue() {
        return turningPidController.getD();
    }

    /*
    public double getSteeringVoltage() { return steeringVoltage; }
    public double getDrivingVoltage() { return drivingVoltage; }
    public void setSteeringVoltage(double volt) { this.steeringVoltage = volt; }
    public void setDrivingVoltage(double volt){this.drivingVoltage=volt;}


     */

    /**
     * The string representation of the swerve module
     * @return "Swerve module side: " + sideNumber: int
     */
    /*public String toString() {
        return "Swerve module side: " + (side+1);
    }*/
    public int getRealSide(){
        int real = side + 1;
        return real;
    }
    public double getSteerRotations(){
        return steerMotor.getPosition().getValueAsDouble();
    }


    public double getSwerveSteeringVoltage() {
        return Constants.Swerve.maxSteerVoltage;
    }
    public void setSwerveSteeringVoltage(double lol) {
        Constants.Swerve.maxSteerVoltage = lol;
    }
    public double getSwerveDrivingVoltage() {
        return Constants.Swerve.maxDriveVoltage;
    }
    public void getSwerveDrivingVoltage(double lol) {
        Constants.Swerve.maxDriveVoltage = lol;
    }

    public double getModuleSteeringSupplyCurrent() { return steerMotor.getSupplyCurrent().getValueAsDouble();}
    public double getModuleDrivingSupplyCurrent() { return driveMotor.getSupplyCurrent().getValueAsDouble();}

    public double getDriveVoltage() {return driveMotor.getMotorVoltage().getValueAsDouble();}
    public double getSteerVoltage() {return steerMotor.getMotorVoltage().getValueAsDouble();}

    public void updateDrivePIDValues() {
        slot0DriveConfigs.kS = drive_kS;
        slot0DriveConfigs.kV = drive_kV;
        slot0DriveConfigs.kA = drive_kA;
        slot0DriveConfigs.kP = drive_kP;
        slot0DriveConfigs.kI = drive_kI;
        slot0DriveConfigs.kD = drive_kD;
        driveMotor.getConfigurator().apply(slot0DriveConfigs);
    }

    public void updateSteerPIDValues() {
        slot0SteerConfigs.kP = steer_kP;
        slot0SteerConfigs.kI = steer_kI;
        slot0SteerConfigs.kD = steer_kD;
        steerMotor.getConfigurator().apply(slot0SteerConfigs);
    }


    public double getDrive_kS() {return drive_kS;}
    public double getDrive_kV() { return drive_kV;}
    public double getDrive_kA() { return drive_kA;}
    public double getDrive_kP() { return drive_kP;}
    public double getDrive_kI() { return drive_kI;}
    public double getDrive_kD() { return drive_kD;}

    public void setDrive_kS(double value) {
        drive_kS = value;}
    public void setDrive_kA(double value) { drive_kA = value;}
    public void setDrive_kV(double value) { drive_kV = value;}
    public void setDrive_kP(double value) { drive_kP = value;}
    public void setDrive_kI(double value) { drive_kI = value;}
    public void setDrive_kD(double value) { drive_kD = value;}

    public double getSteer_kP() { return steer_kP;}
    public double getSteer_kI() { return steer_kI;}
    public double getSteer_kD() { return steer_kD;}

    public void setSteer_kP(double value) { steer_kP = value;}
    public void setSteer_kI(double value) { steer_kI = value;}
    public void setSteer_kD(double value) { steer_kD = value;}

    /**
     * Builds the sendable for shuffleboard
     * @param builder sendable builder
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        if (Constants.debugMode) {
            builder.setSmartDashboardType("Swerve Module " + (getRealSide()));
            builder.addDoubleProperty("Drive velocity", this::getDriveVelocity, null);
            builder.addDoubleProperty("Steer position", this::getSteerRotations, null);
            builder.addDoubleProperty("Steer Angle", this::getSteerPositionWrapped, null);
            builder.addDoubleProperty("Drive position", this::getDrivePosition, null);
            builder.addDoubleProperty("Absolute encoder position", this::getAbsoluteEncoderRots, null);
            builder.addDoubleProperty("Constant Steering voltage", this::getSwerveSteeringVoltage, this::setSwerveSteeringVoltage);
            builder.addDoubleProperty("Constant Driving voltage", this::getSwerveDrivingVoltage, this::getSwerveDrivingVoltage);

            builder.addDoubleProperty("Steering Module Current Supply", this::getModuleSteeringSupplyCurrent, null);
            builder.addDoubleProperty("Driving Module Current Supply", this::getModuleDrivingSupplyCurrent, null);

            //    builder.addDoubleProperty("Steering Voltage", this::getSteeringVoltage, this::setSteeringVoltage);
            // builder.addDoubleProperty("Driving voltage", this:: getDrivingVoltage, this::setDrivingVoltage);
/*        builder.addDoubleProperty("Steer velocity", this::getTurningVelocity, null);
        builder.addDoubleProperty("Steer relative", this::getRelativePositionDegrees, null);
        */
            builder.addDoubleProperty("Turning P " + getRealSide(), this::getPValue, this::setPValue);
            builder.addDoubleProperty("Turning I " + getRealSide(), this::getIValue, this::setIValue);
            builder.addDoubleProperty("Turning D " + getRealSide(), this::getDValue, this::setDValue);

            builder.addDoubleProperty("Drive Velo S " + getRealSide(), this::getDrive_kS, this::setDrive_kS);
            builder.addDoubleProperty("Drive Velo A " + getRealSide(), this::getDrive_kA, this::setDrive_kA);
            builder.addDoubleProperty("Drive Velo V " + getRealSide(), this::getDrive_kV, this::setDrive_kV);
            builder.addDoubleProperty("Drive Velo P " + getRealSide(), this::getDrive_kP, this::setDrive_kP);
            builder.addDoubleProperty("Drive Velo I " + getRealSide(), this::getDrive_kI, this::setDrive_kI);
            builder.addDoubleProperty("Drive Velo D " + getRealSide(), this::getDrive_kD, this::setDrive_kD);

            builder.addDoubleProperty("Steer Velo P " + getRealSide(), this::getSteer_kP, this::setSteer_kP);
            builder.addDoubleProperty("Steer Velo I " + getRealSide(), this::getSteer_kI, this::setSteer_kI);
            builder.addDoubleProperty("Steer Velo D " + getRealSide(), this::getSteer_kD, this::setSteer_kD);


        }
    }

    public double getSteerPositionWrapped() {
        return Math.IEEEremainder(getRelDegrees(), 360);
    }

    public double getRelativePositionDegrees() {
        return Math.toDegrees(getTurningPositionRadians());
    }

    public double getRelDegrees() {
        return Math.toDegrees(getTurningPositionRadians());
    }
}