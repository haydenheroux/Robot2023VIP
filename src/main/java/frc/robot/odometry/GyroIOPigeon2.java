package frc.robot.odometry;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;
import frc.lib.hardware.ConfigurationApplier;

public class GyroIOPigeon2 implements GyroIO {

  private final Pigeon2 gyro;

  private final StatusSignal<Double> roll, pitch, yaw, accelX, accelY, accelZ;

  public GyroIOPigeon2(int id, String canbus) {
    gyro = new Pigeon2(id, canbus);

    roll = gyro.getRoll();
    pitch = gyro.getPitch();
    yaw = gyro.getYaw();

    accelX = gyro.getAccelerationX();
    accelY = gyro.getAccelerationY();
    accelZ = gyro.getAccelerationZ();
  }

  @Override
  public void configure() {
    Pigeon2Configuration gyroConfig = new Pigeon2Configuration();

    gyroConfig.Pigeon2Features.EnableCompass = false;

    ConfigurationApplier.apply(gyroConfig, gyro);
  }

  @Override
  public void updateValues(GyroIOValues values) {
    values.rollAngleRotations = Units.degreesToRotations(roll.getValue());
    values.pitchAngleRotations = Units.degreesToRotations(pitch.getValue());
    values.yawAngleRotations = Units.degreesToRotations(yaw.getValue());

    values.rollAccelerationG = accelX.getValue();
    values.pitchAccelerationG = accelY.getValue();
    values.yawAccelerationG = accelZ.getValue();
  }

  @Override
  public void setYawAngle(double yawAngleRotations) {
    gyro.setYaw(Units.rotationsToDegrees(yawAngleRotations));
  }
}
