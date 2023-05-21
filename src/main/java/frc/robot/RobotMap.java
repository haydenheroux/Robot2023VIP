package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.swerve.ModuleConfiguration.ModuleCAN;

public class RobotMap {

  public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;
  public static final int PNEUMATICS_MODULE = 30;

  public static final int ARM_EXTENSION = 3;
  public static final int ARM_EXTENSION_BRAKE = 10;

  public static final int ARM_ROTATION = 2;
  public static final int ARM_ROTATION_BRAKE = 9;

  public static final int CLAW = 7;

  public static final int BOTTOM_ROLLER = 6;
  public static final int TOP_ROLLER = 5;

  public static final ModuleCAN NORTH_WEST = new ModuleCAN(1, 2, 3, "swerve");
  public static final ModuleCAN NORTH_EAST = new ModuleCAN(4, 5, 6, "swerve");
  public static final ModuleCAN SOUTH_EAST = new ModuleCAN(7, 8, 9, "swerve");
  public static final ModuleCAN SOUTH_WEST = new ModuleCAN(10, 11, 12, "swerve");
}
