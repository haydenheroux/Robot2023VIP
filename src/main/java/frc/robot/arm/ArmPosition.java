package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Arm.Pivot;
import frc.robot.Constants.Arm.Telescoping;
import frc.robot.Constants.Physical;

/**
 * Represents the position of the arm as the point in space occupied by the farthest point of the
 * end effector, relative to the shoulder of the arm.
 */
public class ArmPosition extends Translation2d {

  public static final ArmPosition STOW = ArmPosition.fromValues(0, Pivot.MAX_ANGLE.getRotations());

  public static final ArmPosition SHELF_CONE = new ArmPosition(1.0, Rotation2d.fromDegrees(0));
  public static final ArmPosition SHELF_CUBE = SHELF_CONE.withAngle(Rotation2d.fromDegrees(0));

  public static final ArmPosition SCORE_STANDBY = STOW.withAngle(Rotation2d.fromDegrees(30));

  public static final ArmPosition SCORE_GROUND =
      new ArmPosition(Units.feetToMeters(2.5), Pivot.MIN_ANGLE);
  public static final ArmPosition SCORE_L2 = new ArmPosition(1.15, Rotation2d.fromDegrees(14));
  public static final ArmPosition SCORE_L3 = new ArmPosition(1.7, Rotation2d.fromDegrees(25));

  /**
   * Constructs a new arm position.
   *
   * @param lengthMeters overall length of the arm, in meters.
   * @param angle overall angle of the arm.
   */
  public ArmPosition(double lengthMeters, Rotation2d angle) {
    super(lengthMeters, angle);
  }

  public static ArmPosition fromValues(double sensorLengthMeters, double sensorAngleRotations) {
    return new ArmPosition(
        sensorLengthMeters + Physical.LENGTH_OFFSET,
        Rotation2d.fromRotations(sensorAngleRotations));
  }

  /**
   * Constructs a new arm position derived from the given {@link ArmIO.ArmIOValues}.
   *
   * @param values
   * @return the arm position for the given values.
   */
  public static ArmPosition fromValues(ArmIO.ArmIOValues values) {
    return fromValues(values.telescopingLengthMeters, values.pivotAngleRotations);
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
   * Returns the angle of the arm in this position.
   *
   * @return the angle of the arm in this position.
   */
  @Override
  public Rotation2d getAngle() {
    return super.getAngle();
  }

  /**
   * Returns the angle of the arm in this position according to the rotation sensor, in rotations.
   *
   * @return the angle of the arm in this position according to the rotation sensor, in rotations.
   */
  public double getSensorAngle() {
    return this.getAngle().getRotations();
  }

  /**
   * Returns the angle of the arm in this position, in degrees.
   *
   * @return the angle of the arm in this position, in degrees.
   */
  public double getAngleDegrees() {
    return this.getAngle().getDegrees();
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
   * Returns the length of the arm in this position according to the extension sensor, in meters.
   *
   * @return the length of the arm in this position according to the extension sensor, in meters.
   */
  public double getSensorLength() {
    return this.getLength() - Physical.LENGTH_OFFSET;
  }

  public double getLeverLength() {
    // TODO
    return this.getLength();
  }

  /**
   * Tests if the extension of the arm is at the maximum.
   *
   * @return true if the extension of the arm is at the maximum.
   */
  public boolean lengthAtMax() {
    return getLength() > Telescoping.MAX_LENGTH;
  }

  /**
   * Tests if the extension of the arm is at the minimum.
   *
   * @return true if the extension of the arm is at the minimum.
   */
  public boolean lengthAtMin() {
    return getLength() < Telescoping.MIN_LENGTH;
  }

  /**
   * Tests if the rotation of the arm is at the maximum.
   *
   * @return true if the rotation of the arm is at the maximum.
   */
  public boolean angleAtMax() {
    return getAngle().getRotations() > Pivot.MAX_ANGLE.getRotations();
  }

  /**
   * Tests if the rotation of the arm is at the minimum.
   *
   * @return true if the rotation of the arm is at the minimum.
   */
  public boolean angleAtMin() {
    return getAngle().getRotations() < Pivot.MIN_ANGLE.getRotations();
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

  public boolean isAboveHorizon() {
    return this.getAngle().getRadians() > 0;
  }

  public boolean isBelowHorizon() {
    return this.getAngle().getRadians() < 0;
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
    return Math.abs(difference.getRadians()) < Pivot.TOLERANCE.getRadians();
  }

  /**
   * Returns true if this arm position is at the length of another arm position.
   *
   * @param other
   * @return true if this arm position is at the length of another arm position.
   */
  public boolean atLengthOf(ArmPosition other) {
    return Math.abs(this.getLength() - other.getLength()) < Telescoping.TOLERANCE;
  }

  public boolean isIntersectingGrid() {
    return ArmConstraints.isIntersectingGrid(this);
  }

  public boolean isWithinRuleZone() {
    return ArmConstraints.isWithinRuleZone(this);
  }
}
