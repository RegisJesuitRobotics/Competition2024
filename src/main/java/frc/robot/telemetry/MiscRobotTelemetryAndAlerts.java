package frc.robot.telemetry;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.BuildConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.StringTelemetryEntry;
import frc.robot.telemetry.types.StructTelemetryEntry;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.Metadata;
import frc.robot.utils.RaiderStructs;

public class MiscRobotTelemetryAndAlerts {
  private static final String tableName = "/robot/";

  private final Alert highRIOCanUsageAlert = new Alert("High RIO CAN Usage :(", AlertType.WARNING);
  private final LinearFilter highRIOCanUsageFilter = LinearFilter.movingAverage(50);

  private final Alert highCANivoreCanUsageAlert =
      new Alert("High CANivore CAN Usage :)", AlertType.WARNING);
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

  private final StringTelemetryEntry startCommandsEntry =
      new StringTelemetryEntry(tableName + "startCommands", false);
  private final StringTelemetryEntry endCommandsEntry =
      new StringTelemetryEntry(tableName + "endCommands", false);

  public MiscRobotTelemetryAndAlerts() {
    for (int i = 0; i < controllerAlerts.length; i++) {
      controllerAlerts[i] =
          new Alert(
              "Controller " + MiscConstants.USED_CONTROLLER_PORTS[i] + " is disconnected. :(",
              AlertType.WARNING);
    }

    if (MiscConstants.TUNING_MODE) {
      Alert tuningModeAlert = new Alert("Tuning Mode is Enabled :)", AlertType.INFO);
      tuningModeAlert.set(true);
    }

    CommandScheduler.getInstance()
        .onCommandInitialize((command) -> startCommandsEntry.append(getCommandID(command)));
    CommandScheduler.getInstance()
        .onCommandFinish((command) -> endCommandsEntry.append(getCommandID(command)));
    CommandScheduler.getInstance()
        .onCommandInterrupt((command) -> endCommandsEntry.append(getCommandID(command)));

    // Log build constants to metadata
    Metadata.add("MAVEN_GROUP", BuildConstants.MAVEN_GROUP);
    Metadata.add("MAVEN_NAME", BuildConstants.MAVEN_NAME);
    Metadata.add("VERSION", BuildConstants.VERSION);
    Metadata.add("GIT_REVISION", Integer.toString(BuildConstants.GIT_REVISION));
    Metadata.add("GIT_SHA", BuildConstants.GIT_SHA);
    Metadata.add("GIT_DATE", BuildConstants.GIT_DATE);
    Metadata.add("GIT_BRANCH", BuildConstants.GIT_BRANCH);
    Metadata.add("BUILD_DATE", BuildConstants.BUILD_DATE);
    Metadata.add("BUILD_UNIX_TIME", Long.toString(BuildConstants.BUILD_UNIX_TIME));
    Metadata.add("DIRTY", Integer.toString(BuildConstants.DIRTY));
  }

  private String getCommandID(Command command) {
    return command.getName() + "(" + command.hashCode() + ")";
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
