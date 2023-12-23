package frc.robot.telemetry.wrappers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;

public class TelemetryCANSparkMax extends CANSparkMax {
    private final DoubleTelemetryEntry outputAmpsEntry;
    private final DoubleTelemetryEntry outputPercentEntry;
    private final DoubleTelemetryEntry temperatureEntry;
    private final BooleanTelemetryEntry inBrakeModeEntry;
    private final DoubleTelemetryEntry positionEntry;
    private final DoubleTelemetryEntry velocityEntry;

    public TelemetryCANSparkMax(int deviceId, MotorType type, String telemetryPath, boolean tuningMode) {
        super(deviceId, type);

        telemetryPath += "/";

        outputAmpsEntry = new DoubleTelemetryEntry(telemetryPath + "outputAmps", tuningMode);
        outputPercentEntry = new DoubleTelemetryEntry(telemetryPath + "outputPercent", tuningMode);
        temperatureEntry = new DoubleTelemetryEntry(telemetryPath + "temperature", tuningMode);
        inBrakeModeEntry = new BooleanTelemetryEntry(telemetryPath + "inBrakeMode", tuningMode);
        positionEntry = new DoubleTelemetryEntry(telemetryPath + "position", tuningMode);
        velocityEntry = new DoubleTelemetryEntry(telemetryPath + "velocity", tuningMode);
    }

    @Override
    public REVLibError setIdleMode(IdleMode mode) {
        inBrakeModeEntry.append(mode == IdleMode.kBrake);

        return super.setIdleMode(mode);
    }

    public REVLibError burnFlashWithDelay() {
        Timer.delay(0.1);
        REVLibError error = super.burnFlash();
        Timer.delay(0.1);
        return error;
    }

    public void logValues() {
        outputAmpsEntry.append(super.getOutputCurrent());
        outputPercentEntry.append(super.getAppliedOutput());
        temperatureEntry.append(super.getMotorTemperature());
        positionEntry.append(super.getEncoder().getPosition());
        velocityEntry.append(super.getEncoder().getVelocity());
    }
}
