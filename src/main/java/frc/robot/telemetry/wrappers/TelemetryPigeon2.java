package frc.robot.telemetry.wrappers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.IntegerTelemetryEntry;
import frc.robot.utils.ConfigurationUtils;
import java.util.List;

public class TelemetryPigeon2 extends Pigeon2 {
  private final StatusSignal<Double> yawSignal;
  private final StatusSignal<Double> pitchSignal;
  private final StatusSignal<Double> rollSignal;
  private final StatusSignal<Double> angularVelocityXSignal;
  private final StatusSignal<Double> angularVelocityYSignal;
  private final StatusSignal<Double> angularVelocityZSignal;
  private final StatusSignal<Double> accelXSignal;
  private final StatusSignal<Double> accelYSignal;
  private final StatusSignal<Double> accelZSignal;
  private final StatusSignal<Double> supplyVoltageSignal;
  private final StatusSignal<Double> temperatureSignal;
  private final StatusSignal<Integer> faultSignal;
  private final StatusSignal<Integer> stickyFaultSignal;

  private final DoubleTelemetryEntry yawEntry;
  private final DoubleTelemetryEntry pitchEntry;
  private final DoubleTelemetryEntry rollEntry;
  private final DoubleTelemetryEntry angularVelocityXEntry;
  private final DoubleTelemetryEntry angularVelocityYEntry;
  private final DoubleTelemetryEntry angularVelocityZEntry;
  private final DoubleTelemetryEntry accelXEntry;
  private final DoubleTelemetryEntry accelYEntry;
  private final DoubleTelemetryEntry accelZEntry;
  private final DoubleTelemetryEntry supplyVoltageEntry;
  private final DoubleTelemetryEntry temperatureEntry;
  private final IntegerTelemetryEntry faultEntry;
  private final IntegerTelemetryEntry stickyFaultEntry;

  public TelemetryPigeon2(int deviceId, String telemetryPath, boolean tuningMode) {
    this(deviceId, telemetryPath, "", tuningMode);
  }

  public TelemetryPigeon2(int deviceId, String telemetryPath, String canbus, boolean tuningMode) {
    super(deviceId, canbus);

    // TODO: Check if Device or world
    yawSignal = super.getYaw();
    pitchSignal = super.getPitch();
    rollSignal = super.getRoll();
    angularVelocityXSignal = super.getAngularVelocityXDevice();
    angularVelocityYSignal = super.getAngularVelocityYDevice();
    angularVelocityZSignal = super.getAngularVelocityZDevice();
    accelXSignal = super.getAccelerationX();
    accelYSignal = super.getAccelerationY();
    accelZSignal = super.getAccelerationZ();
    supplyVoltageSignal = super.getSupplyVoltage();
    temperatureSignal = super.getTemperature();
    faultSignal = super.getFaultField();
    stickyFaultSignal = super.getStickyFaultField();

    List.of(
            yawSignal,
            pitchSignal,
            rollSignal,
            angularVelocityXSignal,
            angularVelocityYSignal,
            angularVelocityZSignal,
            accelXSignal,
            accelYSignal,
            accelZSignal,
            supplyVoltageSignal,
            temperatureSignal,
            faultSignal,
            stickyFaultSignal)
        .forEach(ConfigurationUtils::explicitlySetSignalFrequency);

    telemetryPath += "/";
    yawEntry = new DoubleTelemetryEntry(telemetryPath + "yaw", tuningMode);
    pitchEntry = new DoubleTelemetryEntry(telemetryPath + "pitch", tuningMode);
    rollEntry = new DoubleTelemetryEntry(telemetryPath + "roll", tuningMode);
    angularVelocityXEntry =
        new DoubleTelemetryEntry(telemetryPath + "angularVelocityX", tuningMode);
    angularVelocityYEntry =
        new DoubleTelemetryEntry(telemetryPath + "angularVelocityY", tuningMode);
    angularVelocityZEntry =
        new DoubleTelemetryEntry(telemetryPath + "angularVelocityZ", tuningMode);
    accelXEntry = new DoubleTelemetryEntry(telemetryPath + "accelX", tuningMode);
    accelYEntry = new DoubleTelemetryEntry(telemetryPath + "accelY", tuningMode);
    accelZEntry = new DoubleTelemetryEntry(telemetryPath + "accelZ", tuningMode);
    supplyVoltageEntry = new DoubleTelemetryEntry(telemetryPath + "supplyVoltage", tuningMode);
    temperatureEntry = new DoubleTelemetryEntry(telemetryPath + "temperature", tuningMode);
    faultEntry = new IntegerTelemetryEntry(telemetryPath + "faults", tuningMode);
    stickyFaultEntry = new IntegerTelemetryEntry(telemetryPath + "stickyFaults", tuningMode);
  }

  public void logValues() {
    BaseStatusSignal.refreshAll(
        yawSignal,
        pitchSignal,
        rollSignal,
        angularVelocityXSignal,
        angularVelocityYSignal,
        angularVelocityZSignal,
        accelXSignal,
        accelYSignal,
        accelZSignal,
        supplyVoltageSignal,
        temperatureSignal,
        faultSignal,
        stickyFaultSignal);

    yawEntry.append(yawSignal.getValue());
    pitchEntry.append(pitchSignal.getValue());
    rollEntry.append(rollSignal.getValue());
    angularVelocityXEntry.append(angularVelocityXSignal.getValue());
    angularVelocityYEntry.append(angularVelocityYSignal.getValue());
    angularVelocityZEntry.append(angularVelocityZSignal.getValue());
    accelXEntry.append(accelXSignal.getValue());
    accelYEntry.append(accelYSignal.getValue());
    accelZEntry.append(accelZSignal.getValue());
    supplyVoltageEntry.append(supplyVoltageSignal.getValue());
    temperatureEntry.append(temperatureSignal.getValue());
    faultEntry.append(faultSignal.getValue());
    stickyFaultEntry.append(stickyFaultSignal.getValue());
  }
}
