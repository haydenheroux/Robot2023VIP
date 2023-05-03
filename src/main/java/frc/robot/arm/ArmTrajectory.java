// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import frc.robot.Constants;
import java.util.LinkedList;
import java.util.Queue;

public class ArmTrajectory {

  private final Queue<ArmPosition> setpoints;

  public ArmTrajectory(ArmPosition start, ArmPosition end) {
    setpoints = new LinkedList<ArmPosition>();

    boolean needToExtend = start.atLengthOf(end) == false;

    boolean alreadyAvoidingGrid = (start.getLengthMeters() < 0.1 && end.getLengthMeters() < 0.1);
    boolean needToAvoidGrid = !alreadyAvoidingGrid;

    double aboveGridAngleRadians = Constants.Arm.Positions.ABOVE_GRID.getAngleRadians();

    if (needToExtend && needToAvoidGrid) {
      setpoints.add(new ArmPosition(start.getLengthMeters(), aboveGridAngleRadians));
      setpoints.add(new ArmPosition(end.getLengthMeters(), aboveGridAngleRadians));
    }

    setpoints.add(end);
  }

  public ArmPosition get() {
    return setpoints.element();
  }

  public ArmPosition next() {
    boolean hasNext = setpoints.size() > 1;

    if (hasNext) {
      setpoints.remove();
    }

    return get();
  }
}
