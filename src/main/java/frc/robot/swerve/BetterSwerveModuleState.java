// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class BetterSwerveModuleState extends SwerveModuleState {

    private double omegaRadiansPerSecond = 0.0;

    public BetterSwerveModuleState() {
        super();
    }

    public BetterSwerveModuleState(double speedMetersPerSecond, Rotation2d angle, double omegaRadiansPerSecond) {
        super(speedMetersPerSecond, angle);
        this.omegaRadiansPerSecond = omegaRadiansPerSecond;
    }

    public BetterSwerveModuleState(SwerveModuleState state, double omegaRadiansPerSecond) {
        super(state.speedMetersPerSecond, state.angle);
        this.omegaRadiansPerSecond = omegaRadiansPerSecond;
    }

    public static BetterSwerveModuleState optimize(BetterSwerveModuleState desiredState, Rotation2d currentAngle, BetterSwerveModuleState currentState, double omegaFeedforward) {
        if (omegaFeedforward == 0.0) {
            SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, currentAngle);
            return new BetterSwerveModuleState(optimizedState, 0.0);
        }

        Rotation2d postOmegaRadiansPerSecond = Rotation2d.fromRadians(currentState.omegaRadiansPerSecond * omegaFeedforward * 0.065);

        Rotation2d desiredAngle = SwerveMath.placeInAppropriateScope(currentAngle, desiredState.angle.plus(postOmegaRadiansPerSecond));

        double desiredSpeedMetersPerSecond = desiredState.speedMetersPerSecond;
        double deltaDegrees = desiredAngle.minus(currentAngle).getDegrees();

        if (Math.abs(deltaDegrees) > 90) {
            desiredSpeedMetersPerSecond = -desiredSpeedMetersPerSecond;

            final Rotation2d kQuarterTurn = Rotation2d.fromRotations(0.25);

            if (deltaDegrees > 90) {
                desiredAngle = desiredAngle.minus(kQuarterTurn);
            } else {
                desiredAngle = desiredAngle.plus(kQuarterTurn);
            }
        }

        return new BetterSwerveModuleState(desiredSpeedMetersPerSecond, desiredAngle, desiredState.omegaRadiansPerSecond);
    }

}
