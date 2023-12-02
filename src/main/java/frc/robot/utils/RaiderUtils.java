package frc.robot.utils;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.REVLibError;

public class RaiderUtils {
    public static boolean anyTrue(boolean[] array) {
        for (boolean bool : array) {
            if (bool) {
                return true;
            }
        }
        return false;
    }

    public static StatusCode setTalonIdleMode(boolean inBrakeMode, TalonFX talon) {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        StatusCode statusCode = talon.getConfigurator().refresh(motorOutputConfigs);
        if (!statusCode.isOK()) {
            return statusCode;
        }

        motorOutputConfigs.NeutralMode = inBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        return talon.getConfigurator().apply(motorOutputConfigs);
    }

    public static boolean checkRevError(REVLibError code) {
        return code != REVLibError.kOk;
    }
}
