// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import frc.robot.Constants;

public class ArmTrajectory {

    public static ArmPosition next(ArmPosition position, ArmPosition goal) {
        // An angle where, no matter what, it is safe to control the length of the arm
        final double safeAngleRadians = Constants.Arm.Setpoints.SAFE.rotationAngleRadians;

        final boolean atGoalLength = position.lengthEquals(goal);

        final boolean aboveSafeAngle = position.rotationAngleRadians >= safeAngleRadians;
        final boolean isSafeToExtend = aboveSafeAngle;

        // If at the goal length, pivot down to the goal angle
        if (atGoalLength) return goal;

        // Otherwise, the goal length needs to be extended to

        // If possible to safely extend to the goal length now, extend to the goal length now
        if (isSafeToExtend) return new ArmPosition(goal.extensionLengthMeters, safeAngleRadians);

        // Otherwise, pivot to an angle where it is safe to extend
        return new ArmPosition(position.extensionLengthMeters, Constants.Arm.Rotation.MAX_ANGLE);
    }
}
