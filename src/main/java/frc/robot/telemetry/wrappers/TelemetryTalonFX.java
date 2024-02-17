package frc.robot.telemetry.wrappers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.IntegerTelemetryEntry;
import frc.robot.utils.RaiderUtils;
import java.util.List;

public class TelemetryTalonFX extends TalonFX {
  private final StatusSignal<Double> outputAmpsSignal;
  private final StatusSignal<Double> inputAmpsSignal;
  private final StatusSignal<Double> outputPercentSignal;
  private final StatusSignal<Double> temperatureSignal;
  private final StatusSignal<Double> positionSignal;
  private final StatusSignal<Double> velocitySignal;
  private final StatusSignal<Integer> faultsSignal;
  private final StatusSignal<Integer> stickyFaultsSignal;

  private final DoubleTelemetryEntry outputAmpsEntry;
  private final DoubleTelemetryEntry inputAmpsEntry;
  private final DoubleTelemetryEntry outputPercentEntry;
  private final DoubleTelemetryEntry temperatureEntry;
  private final BooleanTelemetryEntry inBrakeModeEntry;
  private final DoubleTelemetryEntry positionEntry;
  private final DoubleTelemetryEntry velocityEntry;
  private final IntegerTelemetryEntry faultsEntry;
  private final IntegerTelemetryEntry stickyFaultsEntry;

  private double loggingPositionConversionFactor = 1.0;
  private double loggingVelocityConversionFactor = 1.0;

  public TelemetryTalonFX(
      int deviceNumber, String telemetryPath, String canbus, boolean tuningMode) {
    super(deviceNumber, canbus);

    outputAmpsSignal = super.getStatorCurrent();
    inputAmpsSignal = super.getSupplyCurrent();
    outputPercentSignal = super.getDutyCycle();
    temperatureSignal = super.getDeviceTemp();
    positionSignal = super.getPosition();
    velocitySignal = super.getVelocity();
    faultsSignal = super.getFaultField();
    stickyFaultsSignal = super.getStickyFaultField();

    // Set the update frequency to the default for all
    List.of(
            outputAmpsSignal,
            inputAmpsSignal,
            outputPercentSignal,
            temperatureSignal,
            positionSignal,
            velocitySignal,
            faultsSignal,
            stickyFaultsSignal)
        .forEach(RaiderUtils::explicitlySetSignalFrequency);

    telemetryPath += "/";
    outputAmpsEntry = new DoubleTelemetryEntry(telemetryPath + "outputAmps", tuningMode);
    inputAmpsEntry = new DoubleTelemetryEntry(telemetryPath + "inputAmps", tuningMode);
    outputPercentEntry = new DoubleTelemetryEntry(telemetryPath + "outputPercent", tuningMode);
    temperatureEntry = new DoubleTelemetryEntry(telemetryPath + "temperature", tuningMode);
    inBrakeModeEntry = new BooleanTelemetryEntry(telemetryPath + "inBrakeMode", tuningMode);
    positionEntry = new DoubleTelemetryEntry(telemetryPath + "position", tuningMode);
    velocityEntry = new DoubleTelemetryEntry(telemetryPath + "velocity", tuningMode);
    faultsEntry = new IntegerTelemetryEntry(telemetryPath + "faults", tuningMode);
    stickyFaultsEntry = new IntegerTelemetryEntry(telemetryPath + "stickyFaults", tuningMode);
  }

  public TelemetryTalonFX(int deviceNumber, String logTable, boolean tuningMode) {
    this(deviceNumber, logTable, "", tuningMode);
  }

  public void setLoggingPositionConversionFactor(double loggingPositionConversionFactor) {
    this.loggingPositionConversionFactor = loggingPositionConversionFactor;
  }

  public void setLoggingVelocityConversionFactor(double loggingVelocityConversionFactor) {
    this.loggingVelocityConversionFactor = loggingVelocityConversionFactor;
  }

  public void logValues() {
    // .refresh() on all
    BaseStatusSignal.waitForAll(
        0.0,
        outputAmpsSignal,
        inputAmpsSignal,
        outputPercentSignal,
        temperatureSignal,
        positionSignal,
        velocitySignal,
        faultsSignal,
        stickyFaultsSignal);

    outputAmpsEntry.append(outputAmpsSignal.getValue());
    inputAmpsEntry.append(inputAmpsSignal.getValue());
    outputPercentEntry.append(outputPercentSignal.getValue());
    temperatureEntry.append(temperatureSignal.getValue());
    positionEntry.append(positionSignal.getValue() * loggingPositionConversionFactor);
    velocityEntry.append(velocitySignal.getValue() * loggingVelocityConversionFactor);
    faultsEntry.append(faultsSignal.getValue());
    stickyFaultsEntry.append(stickyFaultsSignal.getValue());
  }
}
