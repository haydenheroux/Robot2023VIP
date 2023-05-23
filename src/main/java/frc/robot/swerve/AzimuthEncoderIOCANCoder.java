package frc.robot.swerve;

import java.util.Optional;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.MagnetFieldStrength;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.util.Units;

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
    config.sensorDirection = false; //Swerve.INVERTED;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.sensorTimeBase = SensorTimeBase.PerSecond;

    encoder.configAllSettings(config);
  }

  @Override
  public void updateValues(AzimuthEncoderIOValues values) {
    Optional<Double> absoluteAngleRotations = attemptGetAbsoluteAngleRotations();

    if (absoluteAngleRotations.isPresent()) {
      values.absoluteAngleRotations = absoluteAngleRotations.get();
    }
  }

  private Optional<Double> attemptGetAbsoluteAngleRotations()  {
    MagnetFieldStrength health = encoder.getMagnetFieldStrength();

    if (health == MagnetFieldStrength.Invalid_Unknown || health == MagnetFieldStrength.BadRange_RedLED) {
      return Optional.empty();
    }

    double degrees = encoder.getAbsolutePosition();

    if (encoder.getLastError() == ErrorCode.OK) {
      double rotations = Units.degreesToRotations(degrees);
      return Optional.of(rotations);
    }

    return Optional.empty();
  }
}
