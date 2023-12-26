package frc.robot.utils;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

@SuppressWarnings("ClassEscapesDefinedScope")
public class RaiderStructs {
  private RaiderStructs() {}

  public static final CANStatusStruct CANStatusStruct = new CANStatusStruct();

  private static final class CANStatusStruct implements Struct<CANStatus> {
    @Override
    public Class<CANStatus> getTypeClass() {
      return CANStatus.class;
    }

    @Override
    public String getTypeString() {
      return "struct:CANStatus";
    }

    @Override
    public int getSize() {
      return kSizeDouble + kSizeInt32 + kSizeInt32 + kSizeInt32 + kSizeInt32;
    }

    @Override
    public String getSchema() {
      return "double percentBusUtilization;int32 busOffCount;int32 txFullCount;int32 receiveErrorCount;int32 transmitErrorCount";
    }

    @Override
    public CANStatus unpack(ByteBuffer bb) {
      CANStatus status = new CANStatus();
      status.setStatus(bb.getDouble(), bb.getInt(), bb.getInt(), bb.getInt(), bb.getInt());
      return status;
    }

    @Override
    public void pack(ByteBuffer bb, CANStatus value) {
      bb.putDouble(value.percentBusUtilization);
      bb.putInt(value.busOffCount);
      bb.putInt(value.txFullCount);
      bb.putInt(value.receiveErrorCount);
      bb.putInt(value.transmitErrorCount);
    }
  }

  public static final TrapezoidStateStruct trapezoidStateStruct = new TrapezoidStateStruct();

  private static class TrapezoidStateStruct implements Struct<TrapezoidProfile.State> {
    @Override
    public Class<TrapezoidProfile.State> getTypeClass() {
      return TrapezoidProfile.State.class;
    }

    @Override
    public String getTypeString() {
      return "struct:TrapezoidProfile.State";
    }

    @Override
    public int getSize() {
      return kSizeDouble + kSizeDouble;
    }

    @Override
    public String getSchema() {
      return "double position;double velocity";
    }

    @Override
    public TrapezoidProfile.State unpack(ByteBuffer bb) {
      return new TrapezoidProfile.State(bb.getDouble(), bb.getDouble());
    }

    @Override
    public void pack(ByteBuffer bb, TrapezoidProfile.State value) {
      bb.putDouble(value.position);
      bb.putDouble(value.velocity);
    }
  }
}
