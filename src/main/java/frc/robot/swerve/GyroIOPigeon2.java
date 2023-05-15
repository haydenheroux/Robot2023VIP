package frc.robot.swerve;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class GyroIOPigeon2 implements GyroIO {

  private final GyroIO.GyroIOValues values = new GyroIOValues();

  private final WPI_Pigeon2 gyro;

  private Rotation3d offset = new Rotation3d();

  public GyroIOPigeon2(int id, String canbus) {
    gyro = new WPI_Pigeon2(id, canbus);
  }

  @Override
  public void configure() {
    gyro.configFactoryDefault();
    gyro.configEnableCompass(false);
  }

  @Override
  public void updateValues(GyroIOValues values) {
    Rotation3d rotation = getRotation().minus(offset);

    this.values.rollAngleRadians = rotation.getX();
    this.values.pitchAngleRadians = rotation.getY();
    this.values.yawAngleRadians = rotation.getZ();

    Translation3d acceleration = getAcceleration();

    this.values.xAccelerationMetersPerSecondSquared = acceleration.getX();
    this.values.yAccelerationMetersPerSecondSquared = acceleration.getY();
    this.values.zAccelerationMetersPerSecondSquared = acceleration.getZ();

    values = this.values;
  }

  @Override
  public void setRollAngle(double rollAngleRadians) {
    offset =
        new Rotation3d(
            values.rollAngleRadians + rollAngleRadians,
            values.pitchAngleRadians,
            values.yawAngleRadians);
  }

  @Override
  public void setPitchAngle(double pitchAngleRadians) {
    offset =
        new Rotation3d(
            values.rollAngleRadians,
            values.pitchAngleRadians + pitchAngleRadians,
            values.yawAngleRadians);
  }

  @Override
  public void setYawAngle(double yawAngleRadians) {
    offset =
        new Rotation3d(
            values.rollAngleRadians,
            values.pitchAngleRadians,
            values.yawAngleRadians + yawAngleRadians);
  }

  private Rotation3d getRotation() {
    double[] wxyz = new double[4];
    gyro.get6dQuaternion(wxyz);

    return new Rotation3d(new Quaternion(wxyz[0], wxyz[1], wxyz[2], wxyz[3]));
  }

  private Translation3d getAcceleration() {
    short[] ba_xyz = new short[3];
    gyro.getBiasedAccelerometer(ba_xyz);

    return new Translation3d(ba_xyz[0], ba_xyz[1], ba_xyz[2]).times(9.81 / 16384.0);
  }
}
