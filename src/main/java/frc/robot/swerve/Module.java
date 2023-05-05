// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.swerve.AngleMotorIO.AngleMotorIOValues;
import frc.robot.swerve.AzimuthEncoderIO.AzimuthEncoderIOValues;
import frc.robot.swerve.DriveMotorIO.DriveMotorIOValues;

public class Module {

    private final AngleMotorIO angleMotor;
    private final AngleMotorIOValues angleMotorValues = new AngleMotorIOValues();

    private final DriveMotorIO driveMotor;
    private final DriveMotorIOValues driveMotorValues = new DriveMotorIOValues();

    private final AzimuthEncoderIO azimuthEncoder;
    private final AzimuthEncoderIOValues azimuthEncoderValues = new AzimuthEncoderIOValues();

    private final double offsetAngleRadians;

    private BetterSwerveModuleState previousState;

    public Module(ModuleConfiguration config) {
        if (Robot.isSimulation()) {
            angleMotor = null;
            driveMotor = null;
            azimuthEncoder = null;
            offsetAngleRadians = 0.0;
        } else {
            angleMotor = null;
            driveMotor = null;
            azimuthEncoder = null;
            offsetAngleRadians = 0.0;
        }
    }

    public void setDesiredState(BetterSwerveModuleState desiredState, boolean isOpenLoop, boolean isForced) {
        final double kOmegaFeedforward = 0.0; // TODO
        desiredState = BetterSwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(azimuthEncoderValues.absoluteAngleRadians), desiredState, kOmegaFeedforward);

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

        boolean isSameAngle = desiredState.angle.equals(previousState.angle);

        if (isSameAngle == false) {
            angleMotor.setSetpoint(desiredState.angle.getRadians());
        }
    }

}
