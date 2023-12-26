package frc.robot.utils;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;

public class Metadata {
  private static DataLog log;

  public static void init(DataLog log) {
    Metadata.log = log;
  }

  public static void add(String key, String value) {
    StringLogEntry entry = new StringLogEntry(log, "/Metadata/" + key);
    entry.append(value);
  }
}
