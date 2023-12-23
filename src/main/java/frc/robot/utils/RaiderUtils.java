package frc.robot.utils;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.REVLibError;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

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

    /**
     * Set the signal update frequency to the current value (most likely the default). Used to keep one at its default frequency but to make it stay after an optimize call.
     * @param signal The signal
     * @return the status of setting it
     */
    public static StatusCode explicitlySetSignalFrequency(StatusSignal<?> signal) {
        return signal.setUpdateFrequency(signal.getAppliedUpdateFrequency());
    }

    public static boolean applyAndCheck(BooleanSupplier apply, BooleanSupplier check, int attempts) {
        for (int i = 0; i < attempts; i++) {
            if (apply.getAsBoolean() && check.getAsBoolean()) {
                return true;
            }
        }
        return false;
    }

    public static boolean checkRevError(REVLibError code) {
        return code != REVLibError.kOk;
    }
}
