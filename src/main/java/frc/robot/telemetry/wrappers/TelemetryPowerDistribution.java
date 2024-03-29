package frc.robot.telemetry.wrappers;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.telemetry.types.DoubleArrayTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;

public class TelemetryPowerDistribution extends PowerDistribution {
  private static final String tableName = "/power/";

  private final Alert lowBatteryVoltageAlert = new Alert("Low Battery Voltage", AlertType.WARNING);
  private final DoubleTelemetryEntry totalEnergyEntry;
  private final DoubleTelemetryEntry totalPowerEntry;
  private final DoubleTelemetryEntry totalCurrentEntry;
  private final DoubleTelemetryEntry temperatureEntry;
  private final DoubleTelemetryEntry inputVoltageEntry;
  private final DoubleArrayTelemetryEntry channelCurrentEntry;

  double[] currents = new double[getNumChannels()];

  public TelemetryPowerDistribution(int module, ModuleType moduleType) {
    super(module, moduleType);
    super.resetTotalEnergy();

    String thisTableName = tableName + module + "/";
    totalEnergyEntry = new DoubleTelemetryEntry(thisTableName + "totalEnergy", true);
    totalPowerEntry = new DoubleTelemetryEntry(thisTableName + "totalPower", false);
    totalCurrentEntry = new DoubleTelemetryEntry(thisTableName + "totalCurrent", false);
    temperatureEntry = new DoubleTelemetryEntry(thisTableName + "temperature", false);
    inputVoltageEntry = new DoubleTelemetryEntry(thisTableName + "inputVoltage", false);
    channelCurrentEntry = new DoubleArrayTelemetryEntry(thisTableName + "channelCurrents", false);
  }

  public TelemetryPowerDistribution() {
    super.resetTotalEnergy();

    String thisTableName = tableName + "/";
    totalEnergyEntry = new DoubleTelemetryEntry(thisTableName + "totalEnergy", true);
    totalPowerEntry = new DoubleTelemetryEntry(thisTableName + "totalPower", false);
    totalCurrentEntry = new DoubleTelemetryEntry(thisTableName + "totalCurrent", false);
    temperatureEntry = new DoubleTelemetryEntry(thisTableName + "temperature", false);
    inputVoltageEntry = new DoubleTelemetryEntry(thisTableName + "inputVoltage", false);
    channelCurrentEntry = new DoubleArrayTelemetryEntry(thisTableName + "channelCurrents", false);
  }

  public void logValues() {
    totalEnergyEntry.append(super.getTotalEnergy());
    totalPowerEntry.append(super.getTotalPower());
    totalCurrentEntry.append(super.getTotalCurrent());
    temperatureEntry.append(super.getTemperature());
    double voltage = super.getVoltage();
    inputVoltageEntry.append(voltage);

    lowBatteryVoltageAlert.set(voltage < 11.0);

    for (int i = 0; i < currents.length; i++) {
      currents[i] = getCurrent(i);
    }
    channelCurrentEntry.append(currents);
  }
}
