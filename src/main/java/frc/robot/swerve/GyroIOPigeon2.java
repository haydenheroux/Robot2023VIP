package frc.robot.swerve;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.util.Units;

public class GyroIOPigeon2 implements GyroIO {

  private final GyroIO.GyroIOValues values = new GyroIOValues();

  private final WPI_Pigeon2 gyro;

  public GyroIOPigeon2(int id, String canbus) {
    gyro = new WPI_Pigeon2(id, canbus);
  }

  @Override
  public void configure() {
    gyro.configFactoryDefault();

    gyro.zeroGyroBiasNow();
    gyro.configEnableCompass(false);
  }

  @Override
  public void updateValues(GyroIOValues values) {
    double[] rotations = new double[3];
    gyro.getYawPitchRoll(rotations);

    this.values.rollAngleRotations = Units.degreesToRotations(rotations[2]);
    this.values.pitchAngleRotations = Units.degreesToRotations(rotations[1]);
    this.values.yawAngleRotations = Units.degreesToRotations(rotations[0]);

    double[] accelerations = new double[3];
    gyro.getRawGyro(accelerations);

    this.values.rollVelocityRotationsPerSecond = accelerations[0];
    this.values.pitchVelocityRotationsPerSecond = accelerations[1];
    this.values.yawVelocityRotationsPerSecond = accelerations[2];

    values = this.values;
  }

  @Override
  public void setYawAngle(double yawAngleRotations) {
    gyro.setYaw(Units.rotationsToDegrees(yawAngleRotations));
  }

}
