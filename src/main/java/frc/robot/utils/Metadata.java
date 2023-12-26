package frc.robot.utils;

import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.types.StringTelemetryEntry;
import java.util.ArrayList;
import java.util.List;

public class Metadata {
  private static final List<StringTelemetryEntry> entries = new ArrayList<>();

  public static void add(String key, String value) {
    StringTelemetryEntry entry =
        new StringTelemetryEntry("/Metadata/" + key, MiscConstants.TUNING_MODE);
    entry.append(value);

    entries.add(entry);
  }
}
