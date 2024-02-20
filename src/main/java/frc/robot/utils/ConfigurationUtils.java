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
      record.run();
      return true;
    }
    return false;
  }

  public static boolean applyCheckRecord(
      BooleanSupplier apply, BooleanSupplier check, Runnable record, int attempts) {
    if (applyCheck(apply, check, attempts)) {
      record.run();
      return true;
    }
    return false;
  }

  public static boolean applyCheckRecordRev(
      Supplier<REVLibError> apply, BooleanSupplier check, Runnable record, int attempts) {
    if (applyCheckRev(apply, check, attempts)) {
      record.run();
      return true;
    }
    return false;
  }

  public static boolean applyCheckRecordCTRE(
      Supplier<StatusCode> apply, BooleanSupplier check, Runnable record, int attempts) {
    if (applyCheckCTRE(apply, check, attempts)) {
      record.run();
      return true;
    }
    return false;
  }

  public static void postDeviceConfig(
      boolean fault, Consumer<String> subsystemEventEntry, String motorName, String faultString) {
    if (fault) {
      String message = motorName + " failed to initialize: " + faultString;
      subsystemEventEntry.accept(message);
      DriverStation.reportWarning(message, false);
    } else {
      subsystemEventEntry.accept(motorName + " initialized");
    }
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
