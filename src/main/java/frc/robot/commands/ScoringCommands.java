package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SetpointConstants;
import frc.robot.Constants.TransportConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transport.TransportSubsystem;

public class ScoringCommands {
  private ScoringCommands() {}

  public static Command shooterInToleranceCommand(ShooterSubsystem shooterSubsystem) {
    return Commands.waitUntil(shooterSubsystem::inTolerance);
  }

  public static Command shootSetpointShootingCommand(ShooterSubsystem shooterSubsystem) {
    return shooterSubsystem.runVelocityCommand(SetpointConstants.SHOOT_SHOOTER_VELOCITY);
  }

  public static Command shootSetpointZeroCommand(ShooterSubsystem shooterSubsystem) {
    return shooterSubsystem.setVoltageCommand(0.0);
  }

  public static Command shootSetpointIdleCommand(ShooterSubsystem shooterSubsystem) {
    return shooterSubsystem.runVelocityCommand(SetpointConstants.IDLE_SHOOTER_VELOCITY);
  }

  public static Command shootSetpointAmpCommand(ShooterSubsystem shooterSubsystem) {
    return shooterSubsystem.runVelocityCommand(SetpointConstants.AMP_SHOOTER_VELOCITY);
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
