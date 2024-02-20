package frc.robot.telemetry.wrappers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.IntegerTelemetryEntry;

public class TelemetryCANCoder extends CANcoder {
  private final StatusSignal<Double> positionSignal;
  private final StatusSignal<Double> velocitySignal;
  private final StatusSignal<Double> absolutePositionSignal;
  private final StatusSignal<Double> supplyVoltageSignal;
  private final StatusSignal<MagnetHealthValue> magnetHealthSignal;
  private final StatusSignal<Integer> faultSignal;
  private final StatusSignal<Integer> stickyFaultSignal;

  private final DoubleTelemetryEntry positionEntry;
  private final DoubleTelemetryEntry velocityEntry;
  private final DoubleTelemetryEntry absolutePositionEntry;
  private final DoubleTelemetryEntry supplyVoltageEntry;
  private final IntegerTelemetryEntry magnetHealthEntry;
  private final IntegerTelemetryEntry faultEntry;
  private final IntegerTelemetryEntry stickyFaultEntry;

  private double loggingPositionConversionFactor = 1.0;
  private double loggingVelocityConversionFactor = 1.0;

  public TelemetryCANCoder(int deviceId, String telemetryPath, boolean tuningMode) {
    this(deviceId, telemetryPath, "", tuningMode);
  }

  public TelemetryCANCoder(int deviceId, String telemetryPath, String canbus, boolean tuningMode) {
    super(deviceId, canbus);

    positionSignal = super.getPosition();
    velocitySignal = super.getVelocity();
    absolutePositionSignal = super.getAbsolutePosition();
    supplyVoltageSignal = super.getSupplyVoltage();
    magnetHealthSignal = super.getMagnetHealth();
    faultSignal = super.getFaultField();
    stickyFaultSignal = super.getStickyFaultField();

    telemetryPath += "/";
    positionEntry = new DoubleTelemetryEntry(telemetryPath + "position", tuningMode);
    velocityEntry = new DoubleTelemetryEntry(telemetryPath + "velocity", tuningMode);
    absolutePositionEntry =
        new DoubleTelemetryEntry(telemetryPath + "absolutePosition", tuningMode);
    supplyVoltageEntry = new DoubleTelemetryEntry(telemetryPath + "supplyVoltage", tuningMode);
    magnetHealthEntry = new IntegerTelemetryEntry(telemetryPath + "magnetHealth", tuningMode);
    faultEntry = new IntegerTelemetryEntry(telemetryPath + "faults", tuningMode);
    stickyFaultEntry = new IntegerTelemetryEntry(telemetryPath + "stickyFaults", tuningMode);
  }

  public void setLoggingPositionConversionFactor(double factor) {
    loggingPositionConversionFactor = factor;
  }

  public void setLoggingVelocityConversionFactor(double factor) {
    loggingVelocityConversionFactor = factor;
  }

  public void logValues() {
    BaseStatusSignal.refreshAll(
        positionSignal,
        velocitySignal,
        absolutePositionSignal,
        supplyVoltageSignal,
        magnetHealthSignal,
        faultSignal,
        stickyFaultSignal);

    positionEntry.append(positionSignal.getValue() * loggingPositionConversionFactor);
    velocityEntry.append(velocitySignal.getValue() * loggingVelocityConversionFactor);
    absolutePositionEntry.append(
        absolutePositionSignal.getValue() * loggingPositionConversionFactor);
    supplyVoltageEntry.append(supplyVoltageSignal.getValue());
    magnetHealthEntry.append(magnetHealthSignal.getValue().value);
    faultEntry.append(faultSignal.getValue());
    stickyFaultEntry.append(stickyFaultSignal.getValue());
  }
}
