// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveMath {

    public static Rotation2d placeInAppropriateScope(Rotation2d currentAngle, Rotation2d desiredAngle) {
        double currentAngleDegrees = currentAngle.getDegrees();
        double desiredAngleDegrees = desiredAngle.getDegrees();

        double lowerBoundDegrees, upperBoundDegrees;

        double lowerOffsetDegrees = currentAngleDegrees % 360;

        if (lowerOffsetDegrees >= 0) {
            lowerBoundDegrees = currentAngleDegrees - lowerOffsetDegrees;
            upperBoundDegrees = currentAngleDegrees + (360 - lowerOffsetDegrees);
        } else {
            upperBoundDegrees = currentAngleDegrees - lowerOffsetDegrees;
            lowerBoundDegrees = currentAngleDegrees - (360 + lowerOffsetDegrees);
        }

        while (desiredAngleDegrees < lowerBoundDegrees) {
            desiredAngleDegrees += 360;
        }

        while (desiredAngleDegrees > upperBoundDegrees) {
            desiredAngleDegrees -= 360;
        }

        if (desiredAngleDegrees - currentAngleDegrees > 180) {
            desiredAngleDegrees -= 360;
        } else if (desiredAngleDegrees - currentAngleDegrees < -180) {
            desiredAngleDegrees += 360;
        }

        return Rotation2d.fromDegrees(desiredAngleDegrees);
    }
}
