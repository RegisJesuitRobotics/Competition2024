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
import java.util.function.DoubleSupplier;

public class ElevatorWristCommands {
  private ElevatorWristCommands() {}

  public static Command elevatorWristIntakePosition(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    return Commands.parallel(
            elevatorSubsystem.setElevatorPositionCommand(SetpointConstants.INTAKE_ELEVATOR_HEIGHT),
            wristSubsystem.setPositionCommand(SetpointConstants.INTAKE_WRIST_ANGLE_RADIANS))
        .withName("ElevatorWristIntakePosition");
  }

  public static Command elevatorWristInToleranceCommand(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    return Commands.waitUntil(elevatorWristToleranceTrigger(elevatorSubsystem, wristSubsystem))
        .withName("ElevatorWristInTolerance");
  }

  public static Trigger elevatorWristToleranceTrigger(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    return new Trigger(elevatorSubsystem::atGoal).and(wristSubsystem::atGoal);
  }

  public static Command elevatorWristDynamicCommand(DoubleSupplier elevatorSetpoint, DoubleSupplier wristSetpoint, ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    return Commands.parallel(
            elevatorSubsystem.setElevatorPositionCommand(elevatorSetpoint),
            wristSubsystem.setPositionCommand(wristSetpoint))
        .withName("ElevatorWristDynamic");
  }

  public static Command elevatorWristCloseSpeakerCommand(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    return Commands.parallel(
            elevatorSubsystem.setElevatorPositionCommand(
                SetpointConstants.CLOSE_SPEAKER_ELEVATOR_HEIGHT),
            wristSubsystem.setPositionCommand(SetpointConstants.CLOSE_SPEAKER_WRIST_ANGLE_RADIANS))
        .withName("ElevatorWristCloseSpeaker");
  }

  public static Command elevatorWristSafeSpeakerCommand(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    return Commands.parallel(
            elevatorSubsystem.setElevatorPositionCommand(
                SetpointConstants.CLOSE_SPEAKER_ELEVATOR_HEIGHT),
            wristSubsystem.setPositionCommand(SetpointConstants.SAFE_SPEAKER_WRIST_RADIANS))
        .withName("ElevatorWristSafeSpeaker");
  }

  public static Command elevatorWristExpelCommand(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    return Commands.parallel(
            elevatorSubsystem.setElevatorPositionCommand(SetpointConstants.EXPEL_ELEVATOR_HEIGHT),
            wristSubsystem.setPositionCommand(SetpointConstants.EXPEL_WRIST_ANGLE_RADIANS))
        .withName("ElevatorWristExpel");
  }

  public static Command elevatorWristFarSpeakerCommand(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    return Commands.parallel(
            elevatorSubsystem.setElevatorPositionCommand(
                SetpointConstants.FAR_SPEAKER_ELEVATOR_HEIGHT),
            wristSubsystem.setPositionCommand(SetpointConstants.FAR_SPEAKER_WRIST_ANGLE_RADIANS))
        .withName("ElevatorWristFarSpeaker");
  }

  public static Command elevatorWristZeroCommand(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    return Commands.parallel(
            elevatorSubsystem.setElevatorPositionCommand(ElevatorConstants.ELEVATOR_MIN_HEIGHT),
            wristSubsystem.setPositionCommand(WristConstants.WRIST_MIN_RADIANS))
        .withName("ElevatorWristZero");
  }

  public static Command elevatorWristAmpCommand(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    return Commands.parallel(
            elevatorSubsystem.setElevatorPositionCommand(SetpointConstants.AMP_ELEVATOR_HEIGHT),
            wristSubsystem.setPositionCommand(
                SetpointConstants.AMP_WRIST_ANGLE_RADIANS))
        .withName("ElevatorWristAmp");
  }

  public static Command elevatorWristClimbUpCommand(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    return Commands.parallel(
            wristSubsystem.setPositionCommand(
                SetpointConstants.CLIMB_UP_WRIST_ANGLE_RADIANS),
            Commands.sequence(
                Commands.waitUntil(wristSubsystem::atGoal),
                elevatorSubsystem.setElevatorPositionCommand(
                    SetpointConstants.CLIMB_UP_ELEVATOR_HEIGHT)))
        .withName("ElevatorWristClimbUp");
  }

  public static Command elevatorWristClimbDownCommand(
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    return Commands.sequence(
            elevatorSubsystem.setVoltageCommand(-10).until(elevatorSubsystem::atBottomLimit),
            Commands.parallel(
                // TODO: Evaluate stall voltage
                elevatorSubsystem.setVoltageCommand(0.6),
                wristSubsystem.setPositionCommand(SetpointConstants.CLIMB_DOWN_WRIST_ANGLE_RADIANS)))
        .withName("ElevatorWristClimbDown");
  }
}
