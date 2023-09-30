package frc.robot.odometry;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;
import frc.lib.hardware.ConfigurationApplier;
import frc.robot.Constants.Swerve;

public class GyroIOPigeon2 implements GyroIO {

  private final Pigeon2 gyro;

  public GyroIOPigeon2(int id, String canbus) {
    gyro = new Pigeon2(id, canbus);
  }

  @Override
  public void configure() {
    ConfigurationApplier.apply(Swerve.GYRO_CONFIG, gyro);
  }

  @Override
  public void updateValues(GyroIOValues values) {
    values.rollAngleRotations = Units.degreesToRotations(gyro.getRoll().refresh().getValue());
    values.pitchAngleRotations = Units.degreesToRotations(gyro.getPitch().refresh().getValue());
    values.yawAngleRotations = Units.degreesToRotations(gyro.getYaw().refresh().getValue());

    values.rollAccelerationG = gyro.getAccelerationX().getValue();
    values.pitchAccelerationG = gyro.getAccelerationY().getValue();
    values.yawAccelerationG = gyro.getAccelerationZ().getValue();
  }

  @Override
  public void setYawAngle(double yawAngleRotations) {
    gyro.setYaw(Units.rotationsToDegrees(yawAngleRotations));
  }
}
