package frc.robot.telemetry.tunable;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
// Modified from 6328 Mechanical Advantage
import java.util.HashMap;
import java.util.HashMap;




    /**
     * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
     * value not in dashboard.
     */
    public class TunableBooleanEntry {
        private final BooleanEntry networkEntry;
        private final BooleanTelemetryEntry telemetryEntry;
        private final boolean tuningMode;
        private final boolean defaultValue;
        private boolean lastHasChangedValue;

        private HashMap<Integer, Double> hasChangedMap = new HashMap<>();

        /**
         * Create a new TunableNumber with the default value
         *
         * @param networkName Name for network tables
         * @param defaultValue Default value
         * @param tuningMode If false the value will be unchangeable
         */
        public TunableBooleanEntry(String networkName, boolean defaultValue, boolean tuningMode) {
            this.networkEntry =
                    NetworkTableInstance.getDefault().getBooleanTopic(networkName).getEntry(defaultValue);
            // Make sure it gets reset on each deploy
            networkEntry.set(defaultValue);
            this.telemetryEntry = new BooleanTelemetryEntry(networkName, false);
            this.defaultValue = defaultValue;
            this.tuningMode = tuningMode;

            this.lastHasChangedValue = defaultValue;
        }

        /**
         * Get the current value from NT if available
         *
         * @return The current value
         */
        public boolean get() {
            if (!tuningMode) {
                return defaultValue;
            }
            boolean value = networkEntry.get();
            telemetryEntry.append(value);
            return value;
        }

        public void set(boolean value) {
            networkEntry.set(value);
        }

        /**
         * Checks whether the number has changed since our last check
         *
         * @return True if the number has changed since the last time this method was called, false
         *     otherwise
         */
        public boolean hasChanged(int hashCode) {
            double currentValue;
            if (get()){
                currentValue = 1;
            }else{
                currentValue = 0;
            }

            if (hasChangedMap.get(hashCode) == null || currentValue != hasChangedMap.get(hashCode)) {
                hasChangedMap.put(hashCode, currentValue);
                return true;
            }

            return false;
        }
    }


