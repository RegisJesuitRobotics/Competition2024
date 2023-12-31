package frc.robot.telemetry.types;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.util.datalog.ProtobufLogEntry;
import edu.wpi.first.util.protobuf.Protobuf;
import edu.wpi.first.wpilibj.DataLogManager;

public class ProtobufTelemetryEntry<T> extends PrimitiveTelemetryEntry {
  private final ProtobufLogEntry<T> logEntry;
  private final ProtobufPublisher<T> networkPublisher;
  private T lastValue;

  public ProtobufTelemetryEntry(String path, Protobuf<T, ?> Protobuf, boolean shouldNT) {
    this(path, Protobuf, shouldNT, true);
  }

  public ProtobufTelemetryEntry(
      String path, Protobuf<T, ?> Protobuf, boolean shouldNT, boolean shouldLazyLog) {
    super(shouldLazyLog);

    logEntry = ProtobufLogEntry.create(DataLogManager.getLog(), path, Protobuf);
    if (shouldNT) {
      networkPublisher =
          NetworkTableInstance.getDefault().getProtobufTopic(path, Protobuf).publish();
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
