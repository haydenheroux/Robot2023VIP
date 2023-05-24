package frc.lib.hardware;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

/*
 * https://github.com/TitaniumTitans/2023ChargedUp/blob/0306f0274d170ba5cd87808f60e1d64475917b67/src/main/java/frc/robot/subsystems/swerve/module/FalconProModule.java#L201
 */
public class ConfigurationApplier {

  public static StatusCode apply(TalonFXConfiguration config, TalonFX fx) {
    StatusCode status;

    do {
      status = fx.getConfigurator().apply(config);
    } while (!status.isOK());

    return status;
  }

  public static StatusCode apply(CANcoderConfiguration config, CANcoder cc) {
    StatusCode status;

    do {
      status = cc.getConfigurator().apply(config);
    } while (!status.isOK());

    return status;
  }

  public static StatusCode apply(MagnetSensorConfigs config, CANcoder cc) {
    StatusCode status;

    do {
      status = cc.getConfigurator().apply(config);
    } while (!status.isOK());

    return status;
  }

  public static StatusCode apply(Pigeon2Configuration config, Pigeon2 pigeon) {
    StatusCode status;

    do {
      status = pigeon.getConfigurator().apply(config);
    } while (!status.isOK());

    return status;
  }
}
