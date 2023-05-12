package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Arm.Extension;
import frc.robot.Constants.Arm.Rotation;

/**
 * Represents the position of the arm as the point in space occupied by the farthest point of the
 * end effector, relative to the shoulder of the arm.
 */
public class ArmPosition extends Translation2d {

  /**
   * Constructs a new arm position.
   *
   * @param lengthMeters overall length of the arm, in meters.
   * @param angle overall angle of the arm.
   */
  public ArmPosition(double lengthMeters, Rotation2d angle) {
    super(lengthMeters, angle);
  }

  /**
   * Constructs a new arm position derived from the given {@link Arm.State}.
   *
   * @param state
   * @return the arm position for the given state.
   */
  public static ArmPosition fromState(Arm.State state) {
    double length = state.extensionLengthMeters + Extension.LENGTH_OFFSET;
    return new ArmPosition(length, state.rotationAngle);
  }

  /**
   * Constructs a new arm position with the current length, but the angle of another arm position.
   *
   * @param other
   * @return
   */
  public ArmPosition withAngle(ArmPosition other) {
    return this.withAngle(other.getAngle());
  }

  /**
   * Constructs a new arm position with the current angle, but the length of another arm position.
   *
   * @param other
   * @return
   */
  public ArmPosition withLength(ArmPosition other) {
    return this.withLength(other.getLength());
  }

  /**
   * Constructs a new arm position with the current length, but with a different angle.
   *
   * @param angle
   * @return
   */
  public ArmPosition withAngle(Rotation2d angle) {
    return new ArmPosition(this.getLength(), angle);
  }

  /**
   * Constructs a new arm position with the current angle, but with a different length.
   *
   * @param lengthMeters
   * @return
   */
  public ArmPosition withLength(double lengthMeters) {
    return new ArmPosition(lengthMeters, this.getAngle());
  }

  /**
   * Returns the length of the arm in this position, in meters.
   *
   * @return the length of the arm in this position, in meters.
   */
  public double getLength() {
    return this.getNorm();
  }

  /**
   * Returns true if this arm position has an angle greater than the angle of another arm position.
   *
   * @param other
   * @return true if this arm position has an angle greater than the angle of another arm position.
   */
  public boolean isAbove(ArmPosition other) {
    return this.getAngle().getRadians() > other.getAngle().getRadians();
  }

  /**
   * Returns true if this arm position has an angle less than the angle of another arm position.
   *
   * @param other
   * @return true if this arm position has an angle less than the angle of another arm position.
   */
  public boolean isBelow(ArmPosition other) {
    return this.getAngle().getRadians() < other.getAngle().getRadians();
  }

  /**
   * Returns true if this arm position is at the angle and length of another arm position.
   *
   * @param other
   * @return true if this arm position is at the angle and length of another arm position.
   */
  public boolean at(ArmPosition other) {
    return atAngleOf(other) && atLengthOf(other);
  }

  /**
   * Returns true if this arm position is at the angle of another arm position.
   *
   * @param other
   * @return true if this arm position is at the angle of another arm position.
   */
  public boolean atAngleOf(ArmPosition other) {
    Rotation2d difference = this.getAngle().minus(other.getAngle());
    return Math.abs(difference.getRadians()) < Rotation.TOLERANCE.getRadians();
  }

  /**
   * Returns true if this arm position is at the length of another arm position.
   *
   * @param other
   * @return true if this arm position is at the length of another arm position.
   */
  public boolean atLengthOf(ArmPosition other) {
    return Math.abs(this.getLength() - other.getLength()) < Extension.TOLERANCE;
  }

  public boolean isIntersectingGrid() {
    return ArmKinematics.isIntersectingGrid(this);
  }

  public boolean isWithinRuleZone() {
    return ArmKinematics.isWithinRuleZone(this);
  }
}
