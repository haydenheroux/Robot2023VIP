package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.Arm.Positions;
import java.util.LinkedList;
import java.util.Queue;

/**
 * A collection of arm positions that form a safe path between a start position and an end position.
 */
public class ArmTrajectory {

  private final Queue<ArmPosition> setpoints;

  /**
   * Constructs a new trajectory between a start position and an end position.
   *
   * @param start
   * @param end
   */
  public ArmTrajectory(ArmPosition start, ArmPosition end) {
    setpoints = new LinkedList<ArmPosition>();

    setpoints.add(start);

    addSafeExtensionAngle(get(), end);
    addSafeIntermediateTrajectory(get(), end);

    setpoints.add(end);
  }

  /**
   * Returns the current setpoint in the trajectory.
   *
   * @return the current setpoint in the trajectory.
   */
  public ArmPosition get() {
    return setpoints.element();
  }

  /**
   * Advances to and returns the next setpoint in the trajectory.
   *
   * @return the next setpoint in the trajectory.
   */
  public ArmPosition next() {
    boolean hasNext = setpoints.size() > 1;

    if (hasNext) {
      setpoints.remove();
    }

    return get();
  }

  /**
   * Adds a setpoint to the main trajectory to get to a safe extension angle if the main trajectory
   * has extension and the start position is above the safe extension angle.
   *
   * @param start the start position of the sub-trajectory.
   * @param end the end position of the sub-trajectory.
   */
  private void addSafeExtensionAngle(ArmPosition start, ArmPosition end) {
    boolean isExtending = start.getLength() < end.getLength();
    if (start.isBelow(Positions.SAFE) && !isExtending) return;

    setpoints.add(start.withAngle(Positions.SAFE));
  }

  /**
   * Adds a sub-trajectory to the main trajectory to get to a safe position where the next position
   * is the end position of the main trajectory.
   *
   * @param start
   * @param end
   */
  private void addSafeIntermediateTrajectory(ArmPosition start, ArmPosition end) {
    if (directTrajectoryIsSafe(start, end)) return;

    // FIXME Does not work for rotating from below to above
    setpoints.add(start.withAngle(Positions.SAFE));
    setpoints.add(start.withAngle(Positions.SAFE).withLength(end));
  }

  /**
   * Returns true if the direct trajectory between two positions will be safe.
   *
   * @param start the start posiiton of the trajectory.
   * @param end the end position of the trajectory.
   * @return true if the direct trajectory between two positions will be safe.
   */
  private boolean directTrajectoryIsSafe(ArmPosition start, ArmPosition end) {
    Rotation2d startAngle, endAngle;

    // Since interpolating between two angles mandates that start is less than end,
    // initialize the start angle and end angle so that start is less than end
    if (start.isBelow(end)) {
      startAngle = start.getAngle();
      endAngle = end.getAngle();
    } else {
      startAngle = end.getAngle();
      endAngle = start.getAngle();
    }

    // Determine the worst case length for the trajectory
    double worstCaseLength = Math.max(start.getLength(), end.getLength());

    // Sweep through all angles between the start angle and the end angle
    for (double percent = 0.0; percent < 1.0; percent += 0.01) {
      Rotation2d testAngle = startAngle.interpolate(endAngle, percent);
      ArmPosition testPosition = new ArmPosition(worstCaseLength, testAngle);

      if (testPosition.isIntersectingGrid() || !testPosition.isWithinRuleZone()) {
        return false;
      }
    }

    return true;
  }
}
