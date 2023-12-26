package frc.robot.telemetry.types;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DataLogManager;

public class StructTelemetryEntry<T> extends PrimitiveTelemetryEntry {
  private final StructLogEntry<T> logEntry;
  private final StructPublisher<T> networkPublisher;
  private T lastValue;

  public StructTelemetryEntry(String path, Struct<T> struct, boolean shouldNT) {
    this(path, struct, shouldNT, true);
  }

  public StructTelemetryEntry(
      String path, Struct<T> struct, boolean shouldNT, boolean shouldLazyLog) {
    super(shouldLazyLog);

    logEntry = StructLogEntry.create(DataLogManager.getLog(), path, struct);
    if (shouldNT) {
      networkPublisher = NetworkTableInstance.getDefault().getStructTopic(path, struct).publish();
    } else {
      networkPublisher = null;
    }
  }

  /**
   * Appends a value to the log. If lazy logging, only appends if the value is a different object
   * from the last value, and they are not #equals.
   *
   * @param value The value to log
   */
  public void append(T value) {
    if (shouldLog(() -> value.equals(lastValue))) {
      logEntry.append(value);

      if (networkPublisher != null) {
        networkPublisher.set(value);
      }
      lastValue = value;
    }
  }

  @Override
  public void close() {
    logEntry.finish();
    if (networkPublisher != null) {
      networkPublisher.close();
    }
  }
}
