package frc.robot.swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Swerve;

public class AzimuthEncoderIOCANCoder implements AzimuthEncoderIO {

  private final WPI_CANCoder encoder;

  public AzimuthEncoderIOCANCoder(int id, String canbus) {
    encoder = new WPI_CANCoder(id, canbus);
  }

  @Override
  public void configure() {
    encoder.configFactoryDefault();

    CANCoderConfiguration config = new CANCoderConfiguration();

    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    config.sensorDirection = Swerve.INVERTED;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.sensorTimeBase = SensorTimeBase.PerSecond;

    encoder.configAllSettings(config);
  }

  @Override
  public void updateValues(AzimuthEncoderIOValues values) {
    values.absoluteAngleRotations = Units.degreesToRotations(encoder.getAbsolutePosition());
  }
}
