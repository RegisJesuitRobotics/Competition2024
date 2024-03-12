package frc.robot.telemetry.tunable;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.telemetry.tunable.gains.TunablePIDGains;
import frc.robot.telemetry.tunable.gains.TunableTrapezoidalProfileGains;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.StructTelemetryEntry;
import frc.robot.utils.RaiderStructs;

public class TunableTelemetryProfiledPIDController extends ProfiledPIDController {

  private final DoubleTelemetryEntry currentMeasurementEntry;
  private final StructTelemetryEntry<TrapezoidProfile.State> goalEntry;
  private final StructTelemetryEntry<TrapezoidProfile.State> setpointEntry;
  private final DoubleTelemetryEntry outputEntry;
  private final BooleanTelemetryEntry atGoalEntry;

  private final TunablePIDGains pidGains;
  private final TunableTrapezoidalProfileGains profileGains;

  public TunableTelemetryProfiledPIDController(
      String logTable, TunablePIDGains pidGains, TunableTrapezoidalProfileGains profileGains) {
    this(logTable, pidGains, profileGains, 0.02);
  }

  public TunableTelemetryProfiledPIDController(
      String logTable,
      TunablePIDGains pidGains,
      TunableTrapezoidalProfileGains profileGains,
      double period) {
    super(
        pidGains.p.get(),
        pidGains.i.get(),
        pidGains.d.get(),
        new Constraints(profileGains.maxVelocity.get(), profileGains.maxAcceleration.get()),
        period);
    this.pidGains = pidGains;
    this.profileGains = profileGains;

    logTable += "/";
    currentMeasurementEntry = new DoubleTelemetryEntry(logTable + "currentMeasurement", true);
    goalEntry =
        new StructTelemetryEntry<>(logTable + "goal", RaiderStructs.trapezoidStateStruct, true);
    setpointEntry =
        new StructTelemetryEntry<>(logTable + "setpoint", RaiderStructs.trapezoidStateStruct, true);
    outputEntry = new DoubleTelemetryEntry(logTable + "output", true);
    atGoalEntry = new BooleanTelemetryEntry(logTable + "atGoal", true);
  }

  @Override
  public double calculate(double measurement) {
    if (pidGains.hasChanged() || profileGains.hasChanged()) {
      setPID(pidGains.p.get(), pidGains.i.get(), pidGains.d.get());
      setConstraints(
          new Constraints(profileGains.maxVelocity.get(), profileGains.maxAcceleration.get()));
    }

    currentMeasurementEntry.append(measurement);
    goalEntry.append(getGoal());

    double output = super.calculate(measurement);
    setpointEntry.append(getSetpoint());
    outputEntry.append(output);
    atGoalEntry.append(atGoal());

    return output;
  }
}
