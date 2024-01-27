package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.utils.Alert;
import frc.robot.utils.RaiderUtils;

public class ShooterSubsystem extends SubsystemBase {
  // TODO I have no clue what the final cad looks like so these are all arbitrary

<<<<<<< HEAD
  private static int instance = 0;
  private Alert topFlyAlert;
  private final TelemetryCANSparkMax topFly =
      new TelemetryCANSparkMax(
          SHOOTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless, "/shooter/top", true);

  private final TelemetryCANSparkMax topTransport =
      new TelemetryCANSparkMax(
          TOP_TRANSPORT_ID,
=======
  private final TelemetryCANSparkMax motorFly =
      new TelemetryCANSparkMax(
          MOTOR_SHOOTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless, "/shooter/motor", true);
  
  private final TelemetryCANSparkMax motorTransport =
      new TelemetryCANSparkMax(
          MOTOR_TRANSPORT_ID,
>>>>>>> 83696f14c15fa740f27dfef23cc8265f01b9937a
          CANSparkMaxLowLevel.MotorType.kBrushless,
          "/shooterTransport/motor",
          true);
<<<<<<< HEAD
  private RelativeEncoder topFlyEncoder;
=======

  private final RelativeEncoder motorFlyEncoder = motorFly.getEncoder();

>>>>>>> 83696f14c15fa740f27dfef23cc8265f01b9937a
  private final SimpleMotorFeedforward FF =
      new SimpleMotorFeedforward(
          SHOOTER_FF_GAINS.sFF.get(), SHOOTER_FF_GAINS.sFF.get(), SHOOTER_FF_GAINS.vFF.get());

  private final DoubleTelemetryEntry motorFlyVoltageReq =
      new DoubleTelemetryEntry("/shooter/motorVoltage", false);

  private final DoubleTelemetryEntry motorTransportVoltageReq =
      new DoubleTelemetryEntry("/shooterTransport/motorVoltage", false);


  private final DigitalInput shooterFrisbeeSensor = new DigitalInput(SHOOTER_SENSOR);

<<<<<<< HEAD
public ShooterSubsystem(){
  int instanceID = instance++;

  topFlyAlert = new Alert("Module " + instanceID + ": ", Alert.AlertType.ERROR);
}

public void configMotor(){
  topFlyEncoder = topFly.getEncoder();
  boolean faultInitializing = false;
  faultInitializing |= RaiderUtils.applyAndCheckRev(
          () -> topFly.setCANTimeout(250), () -> true, Constants.MiscConstants.CONFIGURATION_ATTEMPTS
  );

  faultInitializing |=
          RaiderUtils.applyAndCheckRev(
                  topFly::restoreFactoryDefaults, () -> true, Constants.MiscConstants.CONFIGURATION_ATTEMPTS
          );
  faultInitializing |=
          RaiderUtils.applyAndCheckRev(
                  () ->
                          topFly.setSmartCurrentLimit(
                                  STALL_MOTOR_CURRENT,
                                  FREE_MOTOR_CURRENT),
                                  () -> true,


                  Constants.MiscConstants.CONFIGURATION_ATTEMPTS
          );
  faultInitializing |=
          RaiderUtils.applyAndCheckRev(
                  () -> topFlyEncoder.setVelocityConversionFactor(SHOOTER_VELOCITY_CONVERSION),
                  () -> topFlyEncoder.getVelocityConversionFactor() == SHOOTER_VELOCITY_CONVERSION,
                  Constants.MiscConstants.CONFIGURATION_ATTEMPTS
          );
  faultInitializing |= RaiderUtils.applyAndCheckRev(
          () -> topFlyEncoder.setPositionConversionFactor(SHOOTER_POSITION_CONVERSION),
          () -> topFlyEncoder.getVelocityConversionFactor() == SHOOTER_POSITION_CONVERSION,
          Constants.MiscConstants.CONFIGURATION_ATTEMPTS
  );
  faultInitializing |=
          RaiderUtils.applyAndCheckRev(
                  () -> topFly.setIdleMode(CANSparkMax.IdleMode.kCoast),
                  () -> topFly.getIdleMode() == CANSparkMax.IdleMode.kCoast,
                  Constants.MiscConstants.CONFIGURATION_ATTEMPTS
          );

  faultInitializing |= RaiderUtils.applyAndCheckRev(
          () -> topFly.setPeriodicFramePeriod(
                  CANSparkMaxLowLevel.PeriodicFrame.kStatus2, (int)(1000 / ODOMETRY_FREQUENCY)),
          () -> true,
          Constants.MiscConstants.CONFIGURATION_ATTEMPTS
          );
  faultInitializing |= RaiderUtils.applyAndCheckRev(
          topFly::burnFlashWithDelay, () -> true, Constants.MiscConstants.CONFIGURATION_ATTEMPTS
  );

  topFlyAlert.set(faultInitializing);

}

  public void setFlyVoltage(double voltage) {
    topFly.setVoltage(voltage);
  }
=======
  private double topVoltage = 0;

  public void setMotorFlyVoltage(double voltage) {
    motorFly.setVoltage(voltage);
  }

>>>>>>> 83696f14c15fa740f27dfef23cc8265f01b9937a

  public void setRPM(int rpm) {
    double forwardVol = FF.calculate(rpm);

<<<<<<< HEAD
    setFlyVoltage(forwardVol);
  }

  public void runShooterTransportIn() {
    topTransport.setVoltage(INTAKE_VOLTAGE);
=======
    setMotorFlyVoltage(forwardVol);
  }

  public void runShooterTransportIn() {
    motorTransport.setVoltage(TRANSPORT_VOLTAGE);
>>>>>>> 83696f14c15fa740f27dfef23cc8265f01b9937a
  }

  public void shooterTransportStop() {
    motorTransport.setVoltage(0);
  }

  public void runShooterTransportOut() {
<<<<<<< HEAD
    topTransport.setVoltage(INTAKE_VOLTAGE);
=======
    motorTransport.setVoltage(-TRANSPORT_VOLTAGE);
>>>>>>> 83696f14c15fa740f27dfef23cc8265f01b9937a
  }

  public void TransportStop() {
    motorTransport.setVoltage(0);
  }

  public boolean isAtSensor() {
    return shooterFrisbeeSensor.get();
  }

  public double getMotorEncoderVelocity() {
    return motorFlyEncoder.getVelocity();
  }
<<<<<<< HEAD
=======

>>>>>>> 83696f14c15fa740f27dfef23cc8265f01b9937a
}
