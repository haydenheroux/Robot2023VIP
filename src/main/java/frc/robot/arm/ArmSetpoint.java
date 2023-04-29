// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

public class ArmSetpoint {

    private static ArmPosition calculateUpwards(ArmPosition position, ArmPosition goal) {
        ArmPosition setpoint = new ArmPosition();

        setpoint.rotationAngleRadians = goal.rotationAngleRadians;

        double maximumLength = ArmConstraintsSolver.calculateMaximumExtensionLength(position.rotationAngleRadians);
        setpoint.extensionLengthMeters = Math.min(goal.extensionLengthMeters, maximumLength);

        return setpoint;
    }

    private static ArmPosition calculateDownwards(ArmPosition position, ArmPosition goal) {
        // TODO
        return goal;
    }

    public static ArmPosition calculate(ArmPosition position, ArmPosition goal) {
        if (position.rotationAngleRadians < goal.rotationAngleRadians) {
            return calculateUpwards(position, goal);
        }

        if (position.rotationAngleRadians > goal.rotationAngleRadians) {
            return calculateDownwards(position, goal);
        }
        
        return goal;
    }
}
