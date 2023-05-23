package frc.robot.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class AzimuthEncoderIOCANCoder implements AzimuthEncoderIO {

  private final CANcoder encoder;

  public AzimuthEncoderIOCANCoder(int id, String canbus) {
    encoder = new CANcoder(id, canbus);
  }

  @Override
  public void configure() {
    CANcoderConfiguration config = new CANcoderConfiguration();

    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    // TODO
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    // TODO
    config.MagnetSensor.MagnetOffset = 0.0;

    encoder.getConfigurator().apply(config);
  }

  @Override
  public void updateValues(AzimuthEncoderIOValues values) {
    values.absoluteAngleRotations = encoder.getAbsolutePosition().getValue();
  }
}
