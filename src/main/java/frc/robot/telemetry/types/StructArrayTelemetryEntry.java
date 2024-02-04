package frc.robot.telemetry.types;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DataLogManager;
import java.util.Arrays;

public class StructArrayTelemetryEntry<T> extends PrimitiveTelemetryEntry {
  private final StructArrayLogEntry<T> logEntry;
  private final StructArrayPublisher<T> networkPublisher;
  private T[] lastValue;

  Struct<T> struct;

  public StructArrayTelemetryEntry(String path, Struct<T> struct, boolean shouldNT) {
    this(path, struct, shouldNT, true);
  }

  public StructArrayTelemetryEntry(
      String path, Struct<T> struct, boolean shouldNT, boolean shouldLazyLog) {
    super(shouldLazyLog);
    this.struct = struct;

    logEntry = StructArrayLogEntry.create(DataLogManager.getLog(), path, struct);
    if (shouldNT) {
      networkPublisher =
          NetworkTableInstance.getDefault().getStructArrayTopic(path, struct).publish();
    } else {
      networkPublisher = null;
    }
  }

  /**
   * Appends a value to the log. If lazy logging, only appends if the value is a different object
   * from the last value, and they are not Arrays#equals.
   *
   * @param value The value to log
   */
  public void append(T[] value) {
    if (shouldLog(() -> Arrays.equals(value, lastValue))) {
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
