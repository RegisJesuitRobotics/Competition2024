package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SetpointConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class ElevatorWristCommands {
  private ElevatorWristCommands() {}

  public static Command elevatorWristIntakePosition(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    return Commands.parallel(
        elevatorSubsystem.setElevatorPositionCommand(SetpointConstants.INTAKE_ELEVATOR_HEIGHT),
        wristSubsystem.setPositonCommand(
            Rotation2d.fromRadians(SetpointConstants.INTAKE_WRIST_ANGLE_RADIANS)));
  }

  public static Command elevatorWristInToleranceCommand(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    return Commands.waitUntil(new Trigger(elevatorSubsystem::atGoal).and(wristSubsystem::atGoal));
  }

  public static Command elevatorWristCloseSpeakerCommand(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    return Commands.parallel(
        elevatorSubsystem.setElevatorPositionCommand(
            SetpointConstants.CLOSE_SPEAKER_ELEVATOR_HEIGHT),
        wristSubsystem.setPositonCommand(
            Rotation2d.fromRadians(SetpointConstants.CLOSE_SPEAKER_WRIST_ANGLE_RADIANS)));
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
        elevatorSubsystem.setElevatorPositionCommand(SetpointConstants.AMP_ELEVATOR_HEIGHT),
        wristSubsystem.setPositonCommand(
            Rotation2d.fromRadians(SetpointConstants.AMP_WRIST_ANGLE_RADIANS)));
  }
}