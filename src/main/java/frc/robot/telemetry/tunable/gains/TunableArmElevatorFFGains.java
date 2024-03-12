package frc.robot.telemetry.tunable.gains;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;

public class TunableArmElevatorFFGains {
  public final TunableDouble sFF;
  public final TunableDouble gFF;
  public final TunableDouble vFF;
  public final TunableDouble aFF;

  /**
   * @param networkName the name to use for network tables
   * @param sFF the arbitrary/static feedforward gain (also known as kS) (in volts)
   * @param gFF the gravity feedforward (also known as kG) (in volts)
   * @param vFF the velocity feedforward gain (also known as kV) (in volts per units/second)
   * @param aFF the acceleration feedforward gain (also known as kA) (in volts units/second^2)
   * @param tuningMode if false the gains will be not be changeable
   */
  public TunableArmElevatorFFGains(
      String networkName, double sFF, double gFF, double vFF, double aFF, boolean tuningMode) {
    networkName += "/";
    this.sFF = new TunableDouble(networkName + "sFF", sFF, tuningMode);
    this.gFF = new TunableDouble(networkName + "gFF", gFF, tuningMode);
    this.vFF = new TunableDouble(networkName + "vFF", vFF, tuningMode);
    this.aFF = new TunableDouble(networkName + "aFF", aFF, tuningMode);
  }

  public boolean hasChanged() {
    return hasChanged(0);
  }

  public boolean hasChanged(int hashCode) {
    return sFF.hasChanged(hashCode)
        || gFF.hasChanged(hashCode)
        || vFF.hasChanged(hashCode)
        || aFF.hasChanged(hashCode);
  }

  public ArmFeedforward createArmFeedforward() {
    return new ArmFeedforward(sFF.get(), gFF.get(), vFF.get(), aFF.get());
  }

  public ElevatorFeedforward createElevatorFeedforward() {
    return new ElevatorFeedforward(sFF.get(), gFF.get(), vFF.get(), aFF.get());
  }
}
