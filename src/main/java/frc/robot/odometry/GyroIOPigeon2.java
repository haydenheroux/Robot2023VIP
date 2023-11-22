package frc.robot.odometry;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;
import frc.lib.hardware.ConfigurationApplier;
import frc.robot.Constants;
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
    values.rollAngleRotations = Units.degreesToRotations(gyro.getRoll().waitForUpdate(Constants.LOOP_TIME).getValue());
    values.pitchAngleRotations = Units.degreesToRotations(gyro.getPitch().waitForUpdate(Constants.LOOP_TIME).getValue());
    values.yawAngleRotations = Units.degreesToRotations(gyro.getYaw().waitForUpdate(Constants.LOOP_TIME).getValue());

    values.rollAccelerationG = gyro.getAccelerationX().waitForUpdate(Constants.LOOP_TIME).getValue();
    values.pitchAccelerationG = gyro.getAccelerationY().waitForUpdate(Constants.LOOP_TIME).getValue();
    values.yawAccelerationG = gyro.getAccelerationZ().waitForUpdate(Constants.LOOP_TIME).getValue();
  }

  @Override
  public void setYawAngle(double yawAngleRotations) {
    gyro.setYaw(Units.rotationsToDegrees(yawAngleRotations));
  }
}
