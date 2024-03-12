package frc.robot.telemetry;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.StructTelemetryEntry;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.Metadata;
import frc.robot.utils.RaiderStructs;

public class MiscRobotTelemetryAndAlerts {
  private static final String tableName = "/robot/";

  private final Alert highRIOCanUsageAlert = new Alert("High RIO CAN Usage", AlertType.WARNING);
  private final LinearFilter highRIOCanUsageFilter = LinearFilter.movingAverage(50);

  private final Alert highCANivoreCanUsageAlert =
      new Alert("High CANivore CAN Usage", AlertType.WARNING);
  private final LinearFilter highCANivoreCanUsageFilter = LinearFilter.movingAverage(50);

  private final Alert[] controllerAlerts = new Alert[MiscConstants.USED_CONTROLLER_PORTS.length];

  private final DoubleTelemetryEntry inputVoltageEntry =
      new DoubleTelemetryEntry(tableName + "inputVoltage", MiscConstants.TUNING_MODE);
  private final DoubleTelemetryEntry inputCurrentEntry =
      new DoubleTelemetryEntry(tableName + "inputCurrent", MiscConstants.TUNING_MODE);
  private final StructTelemetryEntry<CANStatus> canStatusEntry =
      new StructTelemetryEntry<>(
          tableName + "rioCANStatus", RaiderStructs.CANStatusStruct, MiscConstants.TUNING_MODE);
  private final StructTelemetryEntry<CANBusStatus> canivoreStatusEntry =
      new StructTelemetryEntry<>(
          tableName + "canivoreCANStatus",
          RaiderStructs.CANBusStatusStruct,
          MiscConstants.TUNING_MODE);

  public MiscRobotTelemetryAndAlerts() {
    for (int i = 0; i < controllerAlerts.length; i++) {
      controllerAlerts[i] =
          new Alert(
              "Controller " + MiscConstants.USED_CONTROLLER_PORTS[i] + " is disconnected.",
              AlertType.WARNING);
    }

    if (MiscConstants.TUNING_MODE) {
      Alert tuningModeAlert = new Alert("Tuning Mode is Enabled", AlertType.INFO);
      tuningModeAlert.set(true);
    }

   
  }

  public void logValues() {
    inputVoltageEntry.append(RobotController.getInputVoltage());
    inputCurrentEntry.append(RobotController.getInputCurrent());

    // RIO CAN Usage
    CANStatus canStatus = RobotController.getCANStatus();
    canStatusEntry.append(canStatus);
    double percentBusUsage = canStatus.percentBusUtilization;
    double filtered = highRIOCanUsageFilter.calculate(percentBusUsage);
    highRIOCanUsageAlert.set(filtered >= 0.9);

    // CANivore CAN Usage
    CANBusStatus canivoreStatus = CANBus.getStatus(MiscConstants.CANIVORE_NAME);
    canivoreStatusEntry.append(canivoreStatus);
    double percentBusUsageCanivore = canivoreStatus.BusUtilization;
    double filteredCanivore = highCANivoreCanUsageFilter.calculate(percentBusUsageCanivore);
    highCANivoreCanUsageAlert.set(filteredCanivore >= 0.9);

    // Joysticks
    for (int i = 0; i < controllerAlerts.length; i++) {
      controllerAlerts[i].set(
          !DriverStation.isJoystickConnected(MiscConstants.USED_CONTROLLER_PORTS[i]));
    }
  }
}
