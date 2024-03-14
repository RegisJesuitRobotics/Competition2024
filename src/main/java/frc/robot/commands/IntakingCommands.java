package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TransportConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.slapdown.SlapdownSuperstructure;
import frc.robot.subsystems.transport.TransportSubsystem;

public class IntakingCommands {
  private IntakingCommands() {}

  public static Command intakeUntilDetected(
      IntakeSubsystem intake, SlapdownSuperstructure slapdown, TransportSubsystem transport) {
    return Commands.parallel(
            slapdown.setDownAndRunCommand(),
            intake.setIntakeVoltageCommand(IntakeConstants.INTAKE_VOLTAGE),
            transport.setVoltageCommand(TransportConstants.TRANSPORT_LOAD_VOLTAGE))
        .until(transport::atSensor)
        .unless(transport::atSensor)
        .andThen(Commands.parallel(intake.stopCommand(), transport.stopCommand()));
  }

  public static Command intakeUntilDetectedNoSlap(
      IntakeSubsystem intake, TransportSubsystem transport) {
    return Commands.parallel(
            intake.setIntakeVoltageCommand(IntakeConstants.INTAKE_VOLTAGE),
            transport.setVoltageCommand(TransportConstants.TRANSPORT_LOAD_VOLTAGE))
        .until(transport::atSensor)
        .unless(transport::atSensor)
        .andThen(Commands.parallel(intake.stopCommand(), transport.stopCommand()));
  }
}
