package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkFlex;
import frc.robot.utils.Alert;
import frc.robot.utils.RaiderUtils;

public class ShooterSubsystem extends SubsystemBase {
  // TODO I have no clue what the final cad looks like so these are all arbitrary

  private Alert topFlyAlert;

  private final TelemetryCANSparkFlex topFly =
      new TelemetryCANSparkFlex(
          SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless, "/shooter/top", true);

  private RelativeEncoder topFlyEncoder;
  private final SimpleMotorFeedforward FF =
      new SimpleMotorFeedforward(
          SHOOTER_FF_GAINS.sFF.get(), SHOOTER_FF_GAINS.sFF.get(), SHOOTER_FF_GAINS.vFF.get());

  private final DoubleTelemetryEntry topFlyVoltageReq =
      new DoubleTelemetryEntry("/shooter/topVoltage", false);
  private final DoubleTelemetryEntry bottomFlyVoltageReq =
      new DoubleTelemetryEntry("/shooter/bottomVoltage", false);

  private final DigitalInput shooterFrisbeeSensor = new DigitalInput(SHOOTER_SENSOR);

  public ShooterSubsystem() {
    topFlyAlert = new Alert("Shooter: ", Alert.AlertType.ERROR);
    configMotor();
  }

  public void configMotor() {
    topFlyEncoder = topFly.getEncoder();
    boolean faultInitializing = false;
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> topFly.setCANTimeout(250),
            () -> true,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            topFly::restoreFactoryDefaults,
            () -> true,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> topFly.setSmartCurrentLimit(STALL_MOTOR_CURRENT, FREE_MOTOR_CURRENT),
            () -> true,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> topFlyEncoder.setVelocityConversionFactor(SHOOTER_VELOCITY_CONVERSION),
            () -> topFlyEncoder.getVelocityConversionFactor() == SHOOTER_VELOCITY_CONVERSION,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> topFlyEncoder.setPositionConversionFactor(SHOOTER_POSITION_CONVERSION),
            () -> topFlyEncoder.getVelocityConversionFactor() == SHOOTER_POSITION_CONVERSION,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> topFly.setIdleMode(CANSparkMax.IdleMode.kCoast),
            () -> topFly.getIdleMode() == CANSparkMax.IdleMode.kCoast,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            topFly::burnFlashWithDelay, () -> true, Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    topFlyAlert.set(faultInitializing);
  }

  public void setFlyVoltage(double voltage) {
    topFly.setVoltage(voltage);
  }

  public void setRPM(double rpm) {
    double forwardVol = FF.calculate(rpm);

    setFlyVoltage(forwardVol);
  }

  public boolean AtSensor() {
    return shooterFrisbeeSensor.get();
  }

  public Command runFlyRPM(double RPM) {
    return this.run(() -> this.setRPM(RPM));
  }
}