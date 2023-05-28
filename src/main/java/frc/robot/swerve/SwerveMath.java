package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveMath {

  /**
   * Gets the target angle placed in the angle range of the reference angle.
   *
   * @param target target angle, in rotations.
   * @param reference reference angle, in rotations.
   * @return the target angle placed in the angle range of the reference angle.
   */
  public static double placeInClosest0To1Scope(double target, double reference) {
    double reference0, reference1;
    double offset = reference % 1;

    if (offset >= 0) {
      reference0 = reference - offset;
      reference1 = reference + (1 - offset);
    } else {
      reference1 = reference - offset;
      reference0 = reference - (1 + offset);
    }

    while (target++ < reference0) {}
    while (target-- > reference1) {}

    if (target - reference > 0.5) {
      target--;
    } else if (target - reference < -0.5) {
      target++;
    }

    return target;
  }

  /**
   * Calculates the theoretical maximum angular speed achievable by the swerve drive.
   *
   * @param maxSpeedMetersPerSecond the maximum linear speed achievable by the swerve drive.
   * @param farthestModule the module farthest from the pivot point.
   * @return the theoretical maximum angular speed.
   */
  public static Rotation2d calculateTheoreticalMaxAngularSpeed(
      double maxSpeedMetersPerSecond, Translation2d farthestModule) {
    double angularRadius = farthestModule.getNorm();
    return Rotation2d.fromRadians(maxSpeedMetersPerSecond / angularRadius);
  }

  /**
   * Prevents the change in angle of a setpoint at low speeds.
   *
   * @param setpoint the setpoint.
   * @param previousAngle the angle of the previous setpoint.
   * @return the mutated setpoint.
   */
  public static SwerveModuleState dejitter(SwerveModuleState setpoint, Rotation2d previousAngle) {
    if (Math.abs(setpoint.speedMetersPerSecond) < Constants.Swerve.DEJITTER_SPEED) {
      setpoint.angle = previousAngle;
    }

    return setpoint;
  }

  private static Twist2d velocityPoseLog(Pose2d velocity) {
    final var dtheta = velocity.getRotation().getRadians();
    final var halfDtheta = dtheta / 2.0;

    final var cosMinusOne = velocity.getRotation().getCos() - 1;

    double halfThetaByTanOfHalfDtheta;
    if (Math.abs(cosMinusOne) < 1E-9) {
      halfThetaByTanOfHalfDtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    } else {
      halfThetaByTanOfHalfDtheta = -(halfDtheta * velocity.getRotation().getSin()) / cosMinusOne;
    }

    Translation2d translationPart =
        velocity.getTranslation().rotateBy(new Rotation2d(halfThetaByTanOfHalfDtheta, -halfDtheta));

    // TODO
    // YAGSL does **not** perform this multiplication
    // .times(Math.hypot(halfThetaByTanOfHalfDtheta, halfDtheta));

    return new Twist2d(translationPart.getX(), translationPart.getY(), dtheta);
  }

  public static ChassisSpeeds getCorrectedChassisVelocity(ChassisSpeeds chassisVelocity) {
    // https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/5
    Pose2d chassisVelocityPose =
        new Pose2d(
            chassisVelocity.vxMetersPerSecond * Constants.LOOP_TIME,
            chassisVelocity.vyMetersPerSecond * Constants.LOOP_TIME,
            Rotation2d.fromRadians(chassisVelocity.omegaRadiansPerSecond * Constants.LOOP_TIME));

    Twist2d chassisVelocityTwist = SwerveMath.velocityPoseLog(chassisVelocityPose);

    return new ChassisSpeeds(
        chassisVelocityTwist.dx / Constants.LOOP_TIME,
        chassisVelocityTwist.dy / Constants.LOOP_TIME,
        chassisVelocityTwist.dtheta / Constants.LOOP_TIME);
  }
}
