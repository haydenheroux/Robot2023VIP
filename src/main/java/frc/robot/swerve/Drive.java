package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.CustomXboxController;
import frc.lib.controllers.pid.RotationPIDController;
import frc.robot.Constants.Swerve.Drift;
import frc.robot.Constants.Swerve.Theta;
import frc.robot.odometry.Odometry;
import frc.robot.swerve.DriveRequest.TranslationMode;

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
public class Drive extends Command {
  private final Swerve swerve;
  private final Odometry odometry;

  private final CustomXboxController controller;

  private DriveRequest request, previousRequest;
  public final RotationPIDController driftController, thetaController;

  private Rotation2d heading = new Rotation2d();

  public Drive(CustomXboxController controller) {
    this.swerve = Swerve.getInstance();
    this.odometry = Odometry.getInstance();

    addRequirements(this.swerve);

    this.controller = controller;
    this.previousRequest = DriveRequest.fromController(controller);

    driftController = new RotationPIDController(Drift.KP, 0, 0);
    driftController.setSaturation(Drift.SATURATION);

    thetaController = new RotationPIDController(Theta.KP, 0, 0);
    thetaController.setSaturation(Theta.SATURATION);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    request = DriveRequest.fromController(controller);

    Translation2d velocity = request.getRequestedVelocity();

    final Rotation2d yaw = odometry.getRotation();

    if (DriveRequest.startedDrifting(previousRequest, request)) {
      heading = yaw;
    }

    double omegaRadiansPerSecond = 0.0;

    switch (request.rotationMode) {
      case SPINNING:
        omegaRadiansPerSecond = request.getRequestedSpinRate().getRadians();
        break;
      case SNAPPING:
        heading = request.getRequestedSnapAngle();
        omegaRadiansPerSecond = thetaController.calculate(yaw, heading);
        break;
      case DRIFTING:
        omegaRadiansPerSecond = driftController.calculate(yaw, heading);
        break;
    }

    ChassisSpeeds chassisSpeeds;

    if (request.translationMode == TranslationMode.ROBOT_CENTRIC) {
      chassisSpeeds = new ChassisSpeeds(velocity.getX(), velocity.getY(), omegaRadiansPerSecond);
    } else {
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              velocity.getX(), velocity.getY(), omegaRadiansPerSecond, yaw);
    }

    swerve.setChassisSpeeds(chassisSpeeds);

    previousRequest = request;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
