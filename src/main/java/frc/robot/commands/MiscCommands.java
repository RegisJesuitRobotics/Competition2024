package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class MiscCommands {
  private MiscCommands() {}

  public static Command rumbleHIDCommand(GenericHID hid) {
    return Commands.runEnd(
        () -> hid.setRumble(RumbleType.kBothRumble, 1.0),
        () -> hid.setRumble(RumbleType.kBothRumble, 0.0));
  }
}
