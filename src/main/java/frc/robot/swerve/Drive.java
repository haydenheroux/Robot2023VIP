package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.controllers.pid.SaturatedPIDController;
import frc.lib.math.Util;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.Drift;
import frc.robot.Constants.Swerve.Theta;
import frc.robot.odometry.Odometry;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Commands the swerve to drive. This command has the capability to drive in robot-relative and
 * field-relative modes, while rotating at a requested angular velocity or aligning to a requested
 * heading.
 *
 * <p>Translation requests are generated using the X axis and the Y axis of a joystick. Angular
 * velocity requests are generated using one axis of another joystick. Heading requests are
 * generated using the angular velocity axis and the other axis of the angular velocity joystick.
 *
 * <p>"Align" mode is disabled by default. If in "align" mode, heading requests are used. Otherwise,
 * angular velocity requests are used. "Align" mode is toggled by a joystick trigger.
 *
 * <p>"Sniper" mode is disabled by default. If in "sniper" mode, robot-relative mode is used and the
 * translation request is reduced, as well as disabling all angular velocity and heading requests.
 * Otherwise, field-relative mode is used, and other requests are enabled. "Sniper" mode is toggled
 * by a joystick triger.
 *
 * <p>Robot-relative mode directly transforms joystick translation requests into speed setpoints. A
 * forward (+X) translation request results in movement in line with the current yaw.
 *
 * <p>Field-relative mode translates joystick translation requests into field-relative speed
 * setpoints. A forward (+X) translation request results in movement in line with the long (X) axis
 * of the field.
 *
 * <p>Angular velocity requests cause the chassis to rotate around the center of the robot at the
 * requested angular velocity. A positive (+) request results in counter-clockwise (CCW) rotation.
 *
 * <p>Heading requests cause the chassis to rotate until at the requested heading. Heading requests
 * are snapped to a nearby multiple to allow for aligning to common angles.
 */
public class Drive extends CommandBase {
  private final Swerve swerve;

  private final DoubleSupplier forwardsVelocity, sidewaysVelocity, forwardsHeading, sidewaysHeading;
  private final BooleanSupplier align, sniper;
  public final SaturatedPIDController driftThetaController, thetaController;

  private enum TranslationMode {
    FIELD_CENTRIC,
    ROBOT_CENTRIC
  }

  private enum RotationMode {
    DRIFTING,
    SPINNING,
    SNAPPING
  }

  private RotationMode previousRotationMode;
  private Rotation2d setHeading = new Rotation2d();

  private final NetworkTable table;
  private final DoublePublisher requestedOmegaDegreesPerSecondPublisher,
      setHeadingDegreesPublisher,
      headingDeltaDegreesPublisher;
  private final StringPublisher translationModePublisher, rotationModePublisher;

  /**
   * Constructs a new drive command.
   *
   * @param swerve the swerve subsystem to drive with.
   * @param forwardsVelocity the supplier for forwards translation requests.
   * @param sidewaysVelocity the supplier for sideways translation requests.
   * @param forwardsHeading the supplier for the forwards component of heading requests.
   * @param sidewaysHeading the supplier for angular velocity requests and the sideways component of
   *     heading requests.
   * @param align the toggle for "align" mode.
   * @param sniper the toggle for "sniper" mode.
   */
  public Drive(
      Swerve swerve,
      DoubleSupplier forwardsVelocity,
      DoubleSupplier sidewaysVelocity,
      DoubleSupplier forwardsHeading,
      DoubleSupplier sidewaysHeading,
      BooleanSupplier align,
      BooleanSupplier sniper) {
    addRequirements(swerve);

    this.swerve = swerve;
    this.forwardsVelocity = forwardsVelocity;
    this.sidewaysVelocity = sidewaysVelocity;
    this.forwardsHeading = forwardsHeading;
    this.sidewaysHeading = sidewaysHeading;
    this.align = align;
    this.sniper = sniper;

    driftThetaController = new SaturatedPIDController(Drift.KP, 0, 0);
    driftThetaController.enableContinuousInput(-Math.PI, Math.PI);
    driftThetaController.setSaturation(Drift.SATURATION.getRadians());

    thetaController = new SaturatedPIDController(Theta.KP, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setSaturation(Theta.SATURATION.getRadians());

    table = NetworkTableInstance.getDefault().getTable("driveCommand");

    requestedOmegaDegreesPerSecondPublisher =
        table.getDoubleTopic("requestedOmegaDegreesPerSecond").publish();
    setHeadingDegreesPublisher = table.getDoubleTopic("setHeadingDegrees").publish();
    headingDeltaDegreesPublisher = table.getDoubleTopic("headingDeltaDegrees").publish();

    translationModePublisher = table.getStringTopic("translationMode").publish();
    rotationModePublisher = table.getStringTopic("rotationMode").publish();
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    final Translation2d velocity =
        new Translation2d(forwardsVelocity.getAsDouble(), sidewaysVelocity.getAsDouble())
            .times(getVelocityScalar());

    final Translation2d heading =
        new Translation2d(forwardsHeading.getAsDouble(), sidewaysHeading.getAsDouble());

    final TranslationMode translationMode = determineTranslationMode(sniper.getAsBoolean());

    final RotationMode rotationMode = determineRotationMode(heading, align.getAsBoolean());

    final boolean isDrifting = rotationMode == RotationMode.DRIFTING;
    final boolean wasSpinning = previousRotationMode == RotationMode.SPINNING;

    if (isDrifting && wasSpinning) setHeading = Odometry.getInstance().getRotation();

    setHeadingDegreesPublisher.set(setHeading.getDegrees());
    headingDeltaDegreesPublisher.set(
        Odometry.getInstance().getRotation().getDegrees() - setHeading.getDegrees());

    double requestedOmegaRadiansPerSecond = getRequestedOmega(heading, rotationMode);

    requestedOmegaDegreesPerSecondPublisher.set(
        Units.radiansToDegrees(requestedOmegaRadiansPerSecond));

    ChassisSpeeds chassisSpeeds =
        getChassisVelocity(velocity, requestedOmegaRadiansPerSecond, translationMode);

    SwerveModuleState[] setpoints = Constants.Swerve.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    swerve.setSetpoints(setpoints, false);

    translationModePublisher.set(translationMode.toString());
    rotationModePublisher.set(rotationMode.toString());

    previousRotationMode = rotationMode;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  private double getVelocityScalar() {
    if (sniper.getAsBoolean()) return Constants.Swerve.MAX_SPEED * Constants.Swerve.SNIPER_SCALAR;

    return Constants.Swerve.MAX_SPEED;
  }

  private TranslationMode determineTranslationMode(boolean isSniping) {
    if (isSniping) return TranslationMode.ROBOT_CENTRIC;

    return TranslationMode.FIELD_CENTRIC;
  }

  /**
   * Determines if the robot has its heading under control by the driver, or is drifting.
   *
   * @param heading the translation of the heading axis.
   * @param aligning true if aligning to a specific heading.
   * @return true if the robot is drifting.
   */
  private boolean determineIfDrifting(Translation2d heading, boolean aligning) {
    if (aligning) {
      final double kMinHeadingDisplacement = 0.7;

      return heading.getNorm() < kMinHeadingDisplacement;
    }

    final double kOmegaDeadband = 0.1;

    return Math.abs(heading.getY()) < kOmegaDeadband;
  }

  private RotationMode determineRotationMode(Translation2d heading, boolean isAligning) {
    final boolean isDrifting = determineIfDrifting(heading, isAligning);

    if (isDrifting) return RotationMode.DRIFTING;

    if (isAligning) return RotationMode.SNAPPING;

    return RotationMode.SPINNING;
  }

  /**
   * Gets the angular speed for a heading and mode, in radians per second.
   *
   * @param heading the translation of the heading axis.
   * @param aligning true if aligning to a specific heading.
   * @return the angular speed for a heading and mode, in radians per second.
   */
  private double getRequestedOmega(Translation2d heading, RotationMode rotationMode) {
    final double maxAngularSpeedOmegaRadiansPerSecond =
        Constants.Swerve.MAX_ANGULAR_SPEED.getRadians();

    final double yawRadians = Odometry.getInstance().getRotation().getRadians();

    double omegaRadiansPerSecond = 0.0;

    switch (rotationMode) {
      case DRIFTING:
        omegaRadiansPerSecond = driftThetaController.calculate(yawRadians, setHeading.getRadians());
        break;
      case SPINNING:
        omegaRadiansPerSecond = heading.getY() * maxAngularSpeedOmegaRadiansPerSecond;
        break;
      case SNAPPING:
        final double kSnapMultipleDegrees = 90;

        setHeading =
            Util.snapToNearest(heading.getAngle(), Rotation2d.fromDegrees(kSnapMultipleDegrees));

        omegaRadiansPerSecond = thetaController.calculate(yawRadians, setHeading.getRadians());
        break;
    }

    return MathUtil.clamp(
        omegaRadiansPerSecond,
        -maxAngularSpeedOmegaRadiansPerSecond,
        maxAngularSpeedOmegaRadiansPerSecond);
  }

  /**
   * Gets the chassis velocity for requested velocities and mode.
   *
   * @param velocity the requested linear velocity, in meters per second.
   * @param omega the requested angular velocity, in radians per second.
   * @return the chassis velocity.
   */
  private ChassisSpeeds getChassisVelocity(
      Translation2d velocity, double omega, TranslationMode translationMode) {
    if (translationMode == TranslationMode.ROBOT_CENTRIC)
      return new ChassisSpeeds(velocity.getX(), velocity.getY(), 0);

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        velocity.getX(), velocity.getY(), omega, Odometry.getInstance().getRotation());
  }
}
