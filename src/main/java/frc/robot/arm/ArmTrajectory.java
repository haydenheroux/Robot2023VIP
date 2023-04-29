// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import frc.robot.Constants;

public class ArmTrajectory {

    private static ArmPosition calculateUpwards(ArmPosition position, ArmPosition goal) {
        double rotationAngleRadians = goal.rotationAngleRadians;

        double maximumLength = ArmConstraintsSolver.calculateMaximumExtensionLength(position.rotationAngleRadians);
        double extensionLengthMeters = Math.min(goal.extensionLengthMeters, maximumLength);

        return new ArmPosition(extensionLengthMeters, rotationAngleRadians);
    }

    private static ArmPosition calculateDownwards(ArmPosition position, ArmPosition goal) {
        // TODO
        if (position.extensionLengthMeters <= goal.extensionLengthMeters) {
            return goal;
        }
        return Constants.Arm.Setpoints.STOWED;
    }

    public static ArmPosition next(ArmPosition position, ArmPosition setpoint, ArmPosition goal) {
        if (position.rotationAngleRadians < goal.rotationAngleRadians) {
            return calculateUpwards(position, goal);
        }

        if (position.rotationAngleRadians > goal.rotationAngleRadians) {
            return calculateDownwards(position, goal);
        }
        
        return goal;
    }
}
