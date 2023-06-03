package frc.lib.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.arm.Arm;
import frc.robot.arm.ArmPosition;
import frc.robot.intake.Claw;
import frc.robot.intake.SideIntake;

/** Renders robot superstructure using a {@link Mechanism2d}. */
public class Mechanisms {

  private static Mechanisms instance = null;

  /**
   * @param meters size, in meters.
   * @return size, in mechanism units.
   */
  private double metersToMechanism(double meters) {
    return meters;
  }

  /**
   * @param meters size, in meters.
   * @return thickness, in mechanism units.
   */
  private double metersToMechanismThickness(double meters) {
    return 160 * metersToMechanism(meters);
  }

  // Width of mechanism "canvas"
  private final double kMechanismWidth = metersToMechanism(Units.inchesToMeters(48) * 2);
  // Height of mechanism "canvas"
  private final double kMechanismHeight =
      metersToMechanism(Units.feetToMeters(6) + Units.inchesToMeters(6));

  // Colors for static superstructure components
  private final Color8Bit kDefaultColor = new Color8Bit(Color.kGray);

  // Colors for arm superstructure components
  private final Color8Bit kLockedColor = kDefaultColor;
  private final Color8Bit kUnlockedColor = new Color8Bit(Color.kHotPink);
  private final Color8Bit kConstraintColor = new Color8Bit(Color.kOrange);

  // Colors for intake superstructure components
  private final Color8Bit kIntakeDisabledColor = new Color8Bit(Color.kLimeGreen);
  private final Color8Bit kIntakeHoldingConeColor = new Color8Bit(Color.kYellow);
  private final Color8Bit kIntakeHoldingCubeColor = new Color8Bit(Color.kPurple);
  private final Color8Bit kIntakeAcceptingColor = new Color8Bit(Color.kLawnGreen);
  private final Color8Bit kIntakeEjectingColor = new Color8Bit(Color.kOrangeRed);

  // Arm superstructure components
  // Mechanism rending handle; "canvas"
  private final Mechanism2d armMechanism;
  private final MechanismRoot2d armRoot;
  private final MechanismLigament2d armRotation, armExtension, armClaw;

  // Intake superstructure components
  private final Mechanism2d sideIntakeMechanism;
  private final MechanismRoot2d sideIntakeRoot;
  private final MechanismLigament2d sideIntake;

  /** Initializes superstructure components. */
  private Mechanisms() {
    armMechanism = new Mechanism2d(kMechanismWidth, kMechanismHeight);
    sideIntakeMechanism = new Mechanism2d(kMechanismWidth, kMechanismHeight);

    armRoot =
        armMechanism.getRoot(
            "armRoot",
            kMechanismWidth / 2,
            metersToMechanism(Constants.Physical.ARM_SHOULDER.getY()));

    armRotation =
        armRoot.append(
            new MechanismLigament2d(
                "armRotation",
                metersToMechanism(Constants.Physical.ARM_STATIC_SECTION_LENGTH),
                Math.toDegrees(0),
                metersToMechanismThickness(Units.inchesToMeters(2)),
                kLockedColor));

    armExtension =
        armRotation.append(
            new MechanismLigament2d(
                "armExtension",
                0,
                Math.toDegrees(0),
                metersToMechanismThickness(Units.inchesToMeters(1.5)),
                kLockedColor));

    armClaw =
        armExtension.append(
            new MechanismLigament2d(
                "claw",
                metersToMechanism(Constants.Physical.CLAW_LENGTH),
                Math.toDegrees(0),
                metersToMechanismThickness(Units.inchesToMeters(4.717)),
                kIntakeDisabledColor));

    sideIntakeRoot =
        sideIntakeMechanism.getRoot(
            "sideIntakeRoot",
            kMechanismWidth / 2 - metersToMechanism(Units.inchesToMeters(12)) / 2,
            metersToMechanism(Units.inchesToMeters(18)));

    sideIntake =
        sideIntakeRoot.append(
            new MechanismLigament2d(
                "sideIntake",
                metersToMechanism(Units.inchesToMeters(12)),
                Math.toDegrees(0),
                metersToMechanismThickness(Units.inchesToMeters(4)),
                kIntakeDisabledColor));
  }

  public static Mechanisms getInstance() {
    if (instance == null) {
      instance = new Mechanisms();
    }

    return instance;
  }

  public Mechanism2d getArmMechanism() {
    return armMechanism;
  }

  public Mechanism2d getSideIntakeMechanism() {
    return sideIntakeMechanism;
  }

  /**
   * Sets the angle of the arm superstructure component.
   *
   * @param angleRotations angle of the arm.
   */
  private void setAngle(double angleRotations) {
    armRotation.setAngle(Rotation2d.fromRotations(angleRotations));
  }

  /**
   * Sets the length of the arm superstructure component.
   *
   * @param lengthMeters length of the arm, in meters.
   */
  private void setLength(double lengthMeters) {
    armExtension.setLength(metersToMechanism(lengthMeters));
  }

  /**
   * Sets the color of the arm superstructure component depending on brake states.
   *
   * @param isLocked brake states of the arm.
   */
  private void setBrake(Arm.Selector isLocked) {
    switch (isLocked) {
      case kBoth:
        armExtension.setColor(kLockedColor);
        armRotation.setColor(kLockedColor);
        break;
      case kTelescoping:
        armExtension.setColor(kLockedColor);
        armRotation.setColor(kUnlockedColor);
        break;
      case kNeither:
        armExtension.setColor(kUnlockedColor);
        armRotation.setColor(kUnlockedColor);
        break;
      case kPivot:
        armExtension.setColor(kUnlockedColor);
        armRotation.setColor(kLockedColor);
        break;
    }
  }

  /**
   * Sets the color of the arm superstructure component depending on satisfaction of constraints.
   *
   * @param position position of the arm to test constraints against.
   */
  private void setConstraint(ArmPosition position) {
    if (!position.isWithinRuleZone() || position.isIntersectingGrid()) {
      armExtension.setColor(kConstraintColor);
    }
  }

  /**
   * Sets properties of the arm superstructure component to match the properties of the arm.
   *
   * @param position position of the arm.
   * @param isLocked brake states of the arm.
   */
  public void updateArm(ArmPosition position, Arm.Selector isLocked) {
    setAngle(position.getSensorAngle());
    setLength(position.getSensorLength());
    setBrake(isLocked);
    setConstraint(position);
  }

  /**
   * Sets color of the claw superstructure component depending on claw state.
   *
   * @param state claw state.
   */
  public void updateClaw(Claw.State state) {
    switch (state) {
      case kAccepting:
        armClaw.setColor(kIntakeAcceptingColor);
        break;
      case kDisabled:
        armClaw.setColor(kIntakeDisabledColor);
        break;
      case kEjecting:
        armClaw.setColor(kIntakeEjectingColor);
        break;
      case kHolding:
        armClaw.setColor(kIntakeHoldingCubeColor);
        break;
    }
  }

  /**
   * Sets color of the side intake superstructure component depending on side intake state.
   *
   * @param state side intake state.
   */
  public void updateSideIntake(SideIntake.State state) {
    switch (state) {
      case kAccepting:
        sideIntake.setColor(kIntakeAcceptingColor);
        break;
      case kDisabled:
        sideIntake.setColor(kIntakeDisabledColor);
        break;
      case kEjecting:
        sideIntake.setColor(kIntakeEjectingColor);
        break;
      case kHolding:
        sideIntake.setColor(kIntakeHoldingConeColor);
        break;
    }
  }
}
