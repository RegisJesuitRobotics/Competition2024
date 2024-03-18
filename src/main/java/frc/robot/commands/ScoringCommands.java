package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.TransportConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transport.TransportSubsystem;

public class ScoringCommands {
  private ScoringCommands() {}

  public static Command shooterInToleranceCommand(ShooterSubsystem shooterSubsystem) {
    return Commands.waitUntil(shooterSubsystem::inTolerance);
  }

  public static Command shootSetpointCloseSpeakerCommand(ShooterSubsystem shooterSubsystem) {
    return shooterSubsystem.runVelocityCommand(Units.rotationsPerMinuteToRadiansPerSecond(6000.0));
  }

  public static Command shootSetpointFarSpeakerCommand(ShooterSubsystem shooterSubsystem) {
    return shooterSubsystem.runVelocityCommand(Units.rotationsPerMinuteToRadiansPerSecond(6000.0));
  }

  public static Command shootSetpointZeroCommand(ShooterSubsystem shooterSubsystem) {
    return shooterSubsystem.setVoltageCommand(0);
  }

  public static Command shootSetpointIdleCommand(ShooterSubsystem shooterSubsystem) {
    return shooterSubsystem.runVelocityCommand(Units.rotationsPerMinuteToRadiansPerSecond(1500.0));
  }

  public static Command shootSetpointAmpCommand(ShooterSubsystem shooterSubsystem) {
    return shooterSubsystem.runVelocityCommand(Units.rotationsPerMinuteToRadiansPerSecond(2000.0));
  }

  public static Command transportToShooterCommand(TransportSubsystem transportSubsystem) {
    return transportSubsystem.setVoltageCommand(TransportConstants.TRANSPORT_CLOSE_SPEAKER_VOLTAGE);
  }

  public static Command reverseShooterTransportCommand(
      ShooterSubsystem shooterSubsystem, TransportSubsystem transportSubsystem) {
    return Commands.parallel(
            shooterSubsystem.setVoltageCommand(-8.0), transportSubsystem.setVoltageCommand(-8.0))
        .withName("ReverseShooterTransport");
  }
}
