package frc.robot.utils;

import com.ctre.phoenix6.Orchestra;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class OrchestraInstance {
  public static final Orchestra INSTANCE = new Orchestra();

  /**
   * Plays a song using the orchestra. Should probably only be done when disabled.
   *
   * @param filePath The file path to the song
   * @return The command
   */
  public static Command playCommand(String filePath) {
    return Commands.runEnd(INSTANCE::play, INSTANCE::stop)
        .beforeStarting(() -> INSTANCE.loadMusic(filePath))
        .ignoringDisable(true)
        .withName("PlayMusic");
  }
}
