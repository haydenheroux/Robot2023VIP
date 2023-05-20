package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveMath {

  public static Rotation2d calculateMaxAngularSpeed(
      double maxSpeed, ModuleConfiguration farthestModule) {
    double angularRadius = farthestModule.kLocationRelativeToCenterMeters.getNorm();
    return Rotation2d.fromRadians(maxSpeed / angularRadius);
  }

  public static Rotation2d placeInAppropriateScope(
      Rotation2d currentAngle, Rotation2d desiredAngle) {
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

  public static SwerveModuleState dejitter(
      SwerveModuleState state, Rotation2d previousAngle, double minimumSpeed) {
    if (Math.abs(state.speedMetersPerSecond) > minimumSpeed) {
      return state;
    }

    return new SwerveModuleState(state.speedMetersPerSecond, previousAngle);
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
