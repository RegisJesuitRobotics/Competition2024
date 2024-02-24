package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.TransportConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transport.TransportSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class ScoringCommands {
  private ScoringCommands() {}

  public static Command shooterInToleranceCommand(ShooterSubsystem shooterSubsystem) {
    return Commands.waitUntil(shooterSubsystem::inTolerance);
  }

  public static Command shootSetpointCloseSpeakerCommand(ShooterSubsystem shooterSubsystem) {
    return shooterSubsystem.runVelocityCommand(Units.rotationsPerMinuteToRadiansPerSecond(10000.0));
  }

  public static Command shootSetpointZeroCommand(ShooterSubsystem shooterSubsystem) {
    return shooterSubsystem.runVelocityCommand(0);
  }

  public static Command shootSetpointIdleCommand(ShooterSubsystem shooterSubsystem) {
    return shooterSubsystem.runVelocityCommand(Units.rotationsPerMinuteToRadiansPerSecond(5000.0));
  }

  public static Command shootSetpointAmpCommand(ShooterSubsystem shooterSubsystem) {
    return shooterSubsystem.runVelocityCommand(Units.rotationsPerMinuteToRadiansPerSecond(1000.0));
  }

  public static Command transportCloseSpeakerCommand(TransportSubsystem transportSubsystem) {
    return transportSubsystem.setVoltageCommand(TransportConstants.TRANSPORT_CLOSE_SPEAKER_VOLTAGE);
  }

  public static Command elevatorWristInToleranceCommand(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    return Commands.waitUntil(new Trigger(elevatorSubsystem::atGoal).and(wristSubsystem::atGoal));
  }

  public static Command elevatorWristCloseSpeakerCommand(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    return Commands.parallel(
        elevatorSubsystem.setElevatorPositionCommand(Units.inchesToMeters(3)),
        wristSubsystem.setPositonCommand(Rotation2d.fromDegrees(10.0)));
  }

  public static Command elevatorWristZeroCommand(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    return Commands.parallel(
        elevatorSubsystem.setElevatorPositionCommand(ElevatorConstants.ELEVATOR_MIN_HEIGHT),
        wristSubsystem.setPositonCommand(WristConstants.WRIST_MIN));
  }

  public static Command elevatorWristAmpCommand(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    return Commands.parallel(
        elevatorSubsystem.setElevatorPositionCommand(ScoringConstants.AMP_ELEVATOR_HEIGHT),
        wristSubsystem.setPositonCommand(
            Rotation2d.fromRadians(ScoringConstants.AMP_WRIST_ANGLE_RADIANS)));
  }
}
