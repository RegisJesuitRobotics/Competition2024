package frc.robot.utils;

import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

@SuppressWarnings("ClassEscapesDefinedScope")
public class RaiderStructs {
  private RaiderStructs() {}

  public static final CANBusStatusStruct CANBusStatusStruct = new CANBusStatusStruct();

  private static final class CANBusStatusStruct implements Struct<CANBusStatus> {

    @Override
    public Class<CANBusStatus> getTypeClass() {
      return CANBusStatus.class;
    }

    @Override
    public String getTypeString() {
      return "struct:CANBusStatus";
    }

    @Override
    public int getSize() {
      return kSizeInt32 + kSizeFloat + kSizeInt32 + kSizeInt32 + kSizeInt32 + kSizeInt32;
    }

    @Override
    public String getSchema() {
      return "int32 Status;float BusUtilization;int32 BusOffCount;int32 TxFullCount;int32 REC;int32 TEC";
    }

    @Override
    public CANBusStatus unpack(ByteBuffer byteBuffer) {
      CANBusStatus canBusStatus = new CANBusStatus();
      canBusStatus.Status = StatusCode.valueOf(byteBuffer.getInt());
      canBusStatus.BusUtilization = byteBuffer.getFloat();
      canBusStatus.BusOffCount = byteBuffer.getInt();
      canBusStatus.TxFullCount = byteBuffer.getInt();
      canBusStatus.REC = byteBuffer.getInt();
      canBusStatus.TEC = byteBuffer.getInt();
      return canBusStatus;
    }

    @Override
    public void pack(ByteBuffer byteBuffer, CANBusStatus canBusStatus) {
      byteBuffer.putInt(canBusStatus.Status.value);
      byteBuffer.putFloat(canBusStatus.BusUtilization);
      byteBuffer.putInt(canBusStatus.BusOffCount);
      byteBuffer.putInt(canBusStatus.TxFullCount);
      byteBuffer.putInt(canBusStatus.REC);
      byteBuffer.putInt(canBusStatus.TEC);
    }
  }

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
