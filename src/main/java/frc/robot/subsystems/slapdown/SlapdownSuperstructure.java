package frc.robot.subsystems.slapdown;

import static frc.robot.Constants.SlapdownConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SlapdownSuperstructure extends SubsystemBase {
  private final SlapdownFeederSubsystem feederSubsystem = new SlapdownFeederSubsystem();
  private final SlapdownRotationSubsystem rotationSubsystem = new SlapdownRotationSubsystem();

  public SlapdownRotationSubsystem getSlapdownRotationSubsystem() {
    return rotationSubsystem;
  }

  public SlapdownFeederSubsystem getSlapdownFeederSubsystem() {
    return feederSubsystem;
  }

  public Command setDownAndRunCommand() {
    return Commands.parallel(
            feederSubsystem.setVoltageCommand(FEEDER_VOLTAGE),
            rotationSubsystem.setRotationGoalCommand(Rotation2d.fromRadians(ROTATION_DOWN_ANGLE)))
        .withName("SlapdownSetDownAndRun");
  }

  public Command setUpCommand() {
    return Commands.parallel(
            feederSubsystem.setVoltageCommand(0.0),
            rotationSubsystem.setRotationGoalCommand(Rotation2d.fromRadians(ROTATION_UP_ANGLE)))
        .withName("SlapdownSetUp");
  }

  public Command probeRotationHomeCommand() {
    return rotationSubsystem.probeHomeCommand();
  }
}
