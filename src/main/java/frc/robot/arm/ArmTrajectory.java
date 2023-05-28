package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.LinkedList;
import java.util.Queue;

/**
 * A collection of arm ArmPosition that form a safe path between a start position and an end
 * position.
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

    ArmPosition previous = start;

    previous = addSafeBelowHorizonRetraction(start, end);
    previous = addSafeExtensionAngle(previous, end);
    previous = addSafeIntermediateTrajectory(previous, end);

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

  private ArmPosition addSafeBelowHorizonRetraction(ArmPosition start, ArmPosition end) {
    if (start.isAboveHorizon()) return start;
    if (start.isBelowHorizon() && end.isBelowHorizon()) return start;

    ArmPosition setpoint = start.withLength(ArmPosition.SCORE_STANDBY);

    setpoints.add(setpoint);
    return setpoint;
  }

  /**
   * Adds a setpoint to the main trajectory to get to a safe extension angle if the main trajectory
   * has extension and the start position is above the safe extension angle.
   *
   * @param start the start position of the sub-trajectory.
   * @param end the end position of the sub-trajectory.
   * @return the generated setpoint.
   */
  private ArmPosition addSafeExtensionAngle(ArmPosition start, ArmPosition end) {
    boolean isExtending = start.getLength() < end.getLength();
    if (start.isBelow(ArmPosition.SCORE_STANDBY) && !isExtending) return start;

    ArmPosition setpoint = start.withAngle(ArmPosition.SCORE_STANDBY);

    setpoints.add(setpoint);
    return setpoint;
  }

  /**
   * Adds a sub-trajectory to the main trajectory to get to a safe position where the next position
   * is the end position of the main trajectory.
   *
   * @param start
   * @param end
   */
  private ArmPosition addSafeIntermediateTrajectory(ArmPosition start, ArmPosition end) {
    if (directTrajectoryIsSafe(start, end)) return start;

    setpoints.add(start.withAngle(ArmPosition.SCORE_STANDBY));

    ArmPosition setpoint = start.withAngle(ArmPosition.SCORE_STANDBY).withLength(end);
    setpoints.add(setpoint);

    return setpoint;
  }

  /**
   * Returns true if the direct trajectory between two ArmPosition will be safe.
   *
   * @param start the start posiiton of the trajectory.
   * @param end the end position of the trajectory.
   * @return true if the direct trajectory between two ArmPosition will be safe.
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
