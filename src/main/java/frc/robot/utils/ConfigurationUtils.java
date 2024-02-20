package frc.robot.utils;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class ConfigurationUtils {
  /**
   * Set the signal update frequency to the current value (most likely the default). Used to keep
   * one at its default frequency but to make it stay after an optimize call.
   *
   * @param signal The signal
   * @return the status of setting it
   */
  public static StatusCode explicitlySetSignalFrequency(StatusSignal<?> signal) {
    return signal.setUpdateFrequency(signal.getAppliedUpdateFrequency());
  }

  public static boolean applyCheck(Runnable apply, BooleanSupplier check, int attempts) {
    return applyCheck(
        () -> {
          apply.run();
          return true;
        },
        check,
        attempts);
  }

  public static boolean applyCheck(BooleanSupplier apply, BooleanSupplier check, int attempts) {
    for (int i = 0; i < attempts; i++) {
      if (apply.getAsBoolean() && check.getAsBoolean()) {
        return true;
      }
    }
    return false;
  }

  public static boolean applyCheckRev(
      Supplier<REVLibError> apply, BooleanSupplier check, int attempts) {
    return applyCheck(() -> RaiderUtils.isRevOk(apply.get()), check, attempts);
  }

  public static boolean applyCheckCTRE(
      Supplier<StatusCode> apply, BooleanSupplier check, int attempts) {
    return applyCheck(() -> apply.get().isOK(), check, attempts);
  }

  public static boolean applyCheckRecord(
      Runnable apply, BooleanSupplier check, Runnable record, int attempts) {
    if (applyCheck(apply, check, attempts)) {
      return true;
    }
    record.run();
    return false;
  }

  public static boolean applyCheckRecord(
      BooleanSupplier apply, BooleanSupplier check, Runnable record, int attempts) {
    if (applyCheck(apply, check, attempts)) {
      return true;
    }
    record.run();
    return false;
  }

  public static boolean applyCheckRecordRev(
      Supplier<REVLibError> apply, BooleanSupplier check, Runnable record, int attempts) {
    if (applyCheckRev(apply, check, attempts)) {
      return true;
    }
    record.run();
    return false;
  }

  public static boolean applyCheckRecordCTRE(
      Supplier<StatusCode> apply, BooleanSupplier check, Runnable record, int attempts) {
    if (applyCheckCTRE(apply, check, attempts)) {
      return true;
    }
    record.run();
    return false;
  }

  /**
   * Post a device configuration message to the subsystem event entry
   *
   * @param fault true if there was a fault
   * @param subsystemEventEntry a logger for the subsystem
   * @param deviceName the name of the motor
   * @param faultString the fault string
   */
  public static void postDeviceConfig(
      boolean fault, Consumer<String> subsystemEventEntry, String deviceName, String faultString) {
    if (fault) {
      String message = deviceName + " failed to initialize: " + faultString;
      subsystemEventEntry.accept(message);
      DriverStation.reportWarning(message, true);
    } else {
      subsystemEventEntry.accept(deviceName + " initialized");
    }
  }

  /**
   * Compare two doubles for equality within a small epsilon
   *
   * @param d1 the first double
   * @param d2 the second double
   * @return true if the doubles are equal within a small epsilon
   */
  public static boolean fpEqual(double d1, double d2) {
    return Math.abs(d1 - d2) < 0.001;
  }

  public static class StringFaultRecorder {
    private String faultString = "";
    private boolean hasFault = false;

    public String getFaultString() {
      return faultString;
    }

    public boolean hasFault() {
      return hasFault;
    }

    public void append(String fault) {
      hasFault = true;
      faultString += fault + ", ";
    }

    public Runnable run(String fault) {
      return () -> append(fault);
    }
  }
}
