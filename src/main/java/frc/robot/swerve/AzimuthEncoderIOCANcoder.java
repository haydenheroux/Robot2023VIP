package frc.robot.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.lib.hardware.CAN;
import frc.lib.hardware.ConfigurationApplier;
import frc.robot.Constants;

/** Implements azimuth encoder behaviors for a CANCoder. */
public class AzimuthEncoderIOCANcoder implements AzimuthEncoderIO {

  private final CANcoder encoder;
  private final double magnetOffset;

  private final StatusSignal<Double> absolutePosition;

  /**
   * Constructs a new CANcoder azimuth encoder.
   *
   * @param can the CAN for the CANcoder.
   * @param offsetRotations the magnet offset to apply to the CANcoder, in rotations.
   */
  public AzimuthEncoderIOCANcoder(CAN can, double offsetRotations) {
    encoder = new CANcoder(can.id, can.bus);

    absolutePosition = encoder.getAbsolutePosition();

    magnetOffset = offsetRotations;
  }

  @Override
  public void configure() {
    CANcoderConfiguration encoderConfiguration = SwerveFactory.createAzimuthEncoderConfig();

    encoderConfiguration.MagnetSensor.MagnetOffset = magnetOffset;

    ConfigurationApplier.apply(encoderConfiguration, encoder);
  }

  @Override
  public void updateValues(AzimuthEncoderIOValues values) {
    absolutePosition.waitForUpdate(Constants.LOOP_TIME);

    values.angleRotations = absolutePosition.getValue();
  }
}
