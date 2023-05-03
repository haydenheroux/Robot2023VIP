// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.mechanism;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.arm.Arm;
import frc.robot.arm.ArmKinematics;
import frc.robot.arm.ArmPosition;
import frc.robot.arm.ArmState;
import frc.robot.intake.Claw;
import frc.robot.intake.SideIntake;

/** Renders robot superstructure using a {@link Mechanism2d}. */
public class SuperstructureMechanism {

  private static SuperstructureMechanism instance = null;

  /**
   * @param meters size, in meters.
   * @return size, in mechanism units.
   */
  private double metersToMechanism(double meters) {
    return meters * 20;
  }

  /**
   * @param meters size, in meters.
   * @return thickness, in mechanism units.
   */
  private double metersToMechanismThickness(double meters) {
    return 8 * metersToMechanism(meters);
  }

  // Width of mechanism "canvas"
  private final double kMechanismWidth =
      metersToMechanism(Constants.Arm.Constraints.MAX_OUT_LENGTH * 2);
  // Height of mechanism "canvas"
  private final double kMechanismHeight = metersToMechanism(Constants.Arm.Constraints.MAX_HEIGHT);

  // Colors for static superstructure components
  private final Color8Bit kDefaultColor = new Color8Bit(Color.kGray);
  private final Color8Bit kBumpersColor = new Color8Bit(Color.kFirstRed);

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

  // Mechanism rending handle; "canvas"
  private final Mechanism2d mechanism;

  // Static superstructure components
  private final MechanismRoot2d bumpersRoot;
  private final MechanismLigament2d bumpers;

  // Arm superstructure components
  private final MechanismRoot2d armRoot;
  private final MechanismLigament2d armRotation, armExtension, armClaw;

  // Intake superstructure components
  private final MechanismRoot2d sideIntakeRoot;
  private final MechanismLigament2d sideIntake;

  /** Initializes superstructure components. */
  private SuperstructureMechanism() {
    mechanism = new Mechanism2d(kMechanismWidth, kMechanismHeight);

    armRoot =
        mechanism.getRoot(
            "armRoot",
            kMechanismWidth / 2,
            metersToMechanism(Constants.Physical.ARM_SHOULDER_HEIGHT));

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
                4 * metersToMechanism(Units.inchesToMeters(1.5)),
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
        mechanism.getRoot(
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

    bumpersRoot =
        mechanism.getRoot(
            "bumpersRoot",
            (kMechanismWidth / 2) - metersToMechanism(Constants.Physical.BUMPER_DISTANCE),
            metersToMechanism(Units.inchesToMeters(7.5)) / 2);

    bumpers =
        bumpersRoot.append(
            new MechanismLigament2d(
                "bumpers",
                metersToMechanism(2 * Constants.Physical.BUMPER_DISTANCE),
                0,
                metersToMechanismThickness(Units.inchesToMeters(7.5)),
                kBumpersColor));
  }

  public static SuperstructureMechanism getInstance() {
    if (instance == null) {
      instance = new SuperstructureMechanism();
    }

    return instance;
  }

  public Mechanism2d getMechanism() {
    return mechanism;
  }

  /**
   * Sets the angle of the arm superstructure component.
   *
   * @param angleRadians angle of the arm, in radians.
   */
  private void setAngle(double angleRadians) {
    armRotation.setAngle(Math.toDegrees(angleRadians));
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
  private void setBrake(Arm.LockType isLocked) {
    switch (isLocked) {
      case kBoth:
        armExtension.setColor(kLockedColor);
        armRotation.setColor(kLockedColor);
        break;
      case kExtension:
        armExtension.setColor(kLockedColor);
        armRotation.setColor(kUnlockedColor);
        break;
      case kNeither:
        armExtension.setColor(kUnlockedColor);
        armRotation.setColor(kUnlockedColor);
        break;
      case kRotation:
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
    if (ArmKinematics.isWithinRuleZone(position) == false
        || ArmKinematics.isAvoidingGrid(position) == false) {
      armExtension.setColor(kConstraintColor);
    }
  }

  /**
   * Sets properties of the arm superstructure component to match the properties of the arm.
   *
   * @param position position of the arm.
   * @param isLocked brake states of the arm.
   */
  public void updateArm(ArmPosition position, Arm.LockType isLocked) {
    ArmState state = ArmState.fromPosition(position);

    setAngle(state.rotationAngleRadians);
    setLength(state.extensionLengthMeters);
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
      case kHoldingCone:
        armClaw.setColor(kIntakeHoldingConeColor);
        break;
      case kHoldingCube:
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
