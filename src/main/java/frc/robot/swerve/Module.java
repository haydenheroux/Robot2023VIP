// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.swerve.AngleMotorIO.AngleMotorIOValues;
import frc.robot.swerve.AzimuthEncoderIO.AzimuthEncoderIOValues;
import frc.robot.swerve.DriveMotorIO.DriveMotorIOValues;

public class Module {

    private final ModuleConfiguration config;

    private final AngleMotorIO angleMotor;
    private final AngleMotorIOValues angleMotorValues = new AngleMotorIOValues();

    private final DriveMotorIO driveMotor;
    private final DriveMotorIOValues driveMotorValues = new DriveMotorIOValues();

    private final AzimuthEncoderIO azimuthEncoder;
    private final AzimuthEncoderIOValues azimuthEncoderValues = new AzimuthEncoderIOValues();

    private final double offsetAngleRadians;

    private SwerveModuleState state;

    public Module(ModuleConfiguration config) {
        if (Robot.isSimulation()) {
            angleMotor = null;
            driveMotor = null;
            azimuthEncoder = null;
        } else {
            angleMotor = null;
            driveMotor = null;
            azimuthEncoder = null;
        }
        
        this.config = config;
        offsetAngleRadians = 0.0; // TODO

        angleMotor.configure();
        driveMotor.configure();

        azimuthEncoder.configure();

        azimuthEncoder.updateValues(azimuthEncoderValues);
        angleMotor.setPosition(azimuthEncoderValues.absoluteAngleRadians - offsetAngleRadians);

        state = getState();
    }

    public void update() {
        angleMotor.updateValues(angleMotorValues);
        driveMotor.updateValues(driveMotorValues);

        azimuthEncoder.updateValues(azimuthEncoderValues);

        state = getState();

        // TODO angleMotor.setPosition({azimuthAngle})
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean isForced) {
        desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(angleMotorValues.angleRadians));

        if (isOpenLoop) {
            final double kMaxSpeedMetersPerSecond = 0.0; // TODO
            double percent = desiredState.speedMetersPerSecond / kMaxSpeedMetersPerSecond;
            double volts = percent * Constants.NOMINAL_VOLTAGE;
            driveMotor.setVoltage(volts);
        } else {
            driveMotor.setVelocitySetpoint(desiredState.speedMetersPerSecond);
        }

        if (isForced == false) {
            // TODO anti-jitter
        }

        boolean isSameAngle = desiredState.angle.equals(state.angle);

        if (isSameAngle == false) {
            angleMotor.setSetpoint(desiredState.angle.getRadians());
        }

        state = desiredState;
    }

    public Translation2d getLocation() {
        return config.locationRelativeToCenterMeters;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotorValues.velocityMetersPerSecond, Rotation2d.fromRadians(angleMotorValues.angleRadians));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotorValues.positionMeters, Rotation2d.fromRadians(angleMotorValues.angleRadians));
    }

    public void setMotorBrake(boolean isActive) {
        driveMotor.setBrake(isActive);
    }

    public void zeroDrivePosition() {
        driveMotor.setPosition(0);
    }
}
