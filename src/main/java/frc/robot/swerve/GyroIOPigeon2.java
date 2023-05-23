package frc.robot.swerve;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;

public class GyroIOPigeon2 implements GyroIO {

  private final Pigeon2 gyro;

  public GyroIOPigeon2(int id, String canbus) {
    gyro = new Pigeon2(id, canbus);
  }

  @Override
  public void configure() {
    Pigeon2Configuration config = new Pigeon2Configuration();

    config.Pigeon2Features.EnableCompass = false;

    gyro.getConfigurator().apply(config);

    // TODO
    // gyro.zeroGyroBiasNow();
    gyro.setYaw(0);

    gyro.getRoll().setUpdateFrequency(100);
    gyro.getPitch().setUpdateFrequency(100);
    gyro.getYaw().setUpdateFrequency(100);

    gyro.getAccelerationX().setUpdateFrequency(100);
    gyro.getAccelerationY().setUpdateFrequency(100);
    gyro.getAccelerationZ().setUpdateFrequency(100);
  }

  @Override
  public void updateValues(GyroIOValues values) {
    values.rollAngleRotations = Units.degreesToRotations(gyro.getRoll().getValue());
    values.pitchAngleRotations = Units.degreesToRotations(gyro.getPitch().getValue());
    values.yawAngleRotations = Units.degreesToRotations(gyro.getYaw().getValue());

    values.rollAccelerationG = gyro.getAccelerationX().getValue();
    values.pitchAccelerationG = gyro.getAccelerationY().getValue();
    values.yawAccelerationG = gyro.getAccelerationZ().getValue();
  }

  @Override
  public void setYawAngle(double yawAngleRotations) {
    gyro.setYaw(Units.rotationsToDegrees(yawAngleRotations));
  }
}
