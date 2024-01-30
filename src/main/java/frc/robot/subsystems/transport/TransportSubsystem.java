package frc.robot.subsystems.transport;

import static frc.robot.Constants.TransportConstants.*;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;

public class TransportSubsystem extends SubsystemBase {
  private static int instance = 0;
  public final TelemetryCANSparkMax transportMotor =
      new TelemetryCANSparkMax(
          TRANSPORT_MOTOR_ID,
          CANSparkMaxLowLevel.MotorType.kBrushless,
          "/motors/intake",
          TUNING_MODE);

          private final TelemetryCANSparkMax topTransport =
      new TelemetryCANSparkMax(
          TOP_TRANSPORT_ID,
          CANSparkMaxLowLevel.MotorType.kBrushless,
          "/shooterTransport/top",
          true);

          public void runShooterTransportVoltage(double voltage){
            topTransport.setVoltage(voltage);
          }
        
          public void TransportStop() {
            topTransport.setVoltage(0);
          }

      private final DoubleTelemetryEntry topTransportVoltageReq =
          new DoubleTelemetryEntry("/shooterTransport/topVoltage", false);
      private final DoubleTelemetryEntry bottomTransportVoltageReq =
          new DoubleTelemetryEntry("/shooterTransport/bottomVoltage", false);
          
public TransportSubsystem(){
  instanceID = istance++;
  transportAlert = new Alert("Module " + instanceID + ": ", Alert.AlertType.ERROR);
  configMotor();
}

public void configMotor(){
 boolean faultInitializing = false;
  faultInitializing |= RaiderUtils.applyAndCheckRev(
          () -> transportMotor.setCANTimeout(250), () -> true, Constants.MiscConstants.CONFIGURATION_ATTEMPTS
  );

  faultInitializing |=
       RaiderUtils.applyAndCheckRev(
             transportMotor::restoreFactoryDefaults, () -> true, Constants.MiscConstants.CONFIGURATION_ATTEMPTS
          );

  faultInitializing |=
       RaiderUtils.applyAndCheckRev(
           () ->
               transportMotor.setSmartCurrentLimit(
                   STALL_MOTOR_CURRENT,
                   FREE_MOTOR_CURRENT),
                        () -> true,


                  Constants.MiscConstants.CONFIGURATION_ATTEMPTS
          );
        
  faultInitializing |=
          RaiderUtils.applyAndCheckRev(
               () -> transportMotor.setIdleMode(CANSparkMax.IdleMode.kCoast),
               () -> transportMotor.getIdleMode() == CANSparkMax.IdleMode.kCoast,
                 Constants.MiscConstants.CONFIGURATION_ATTEMPTS
          );

  faultInitializing |= RaiderUtils.applyAndCheckRev(
          () -> transportMotor.setPeriodicFramePeriod(
                  CANSparkMaxLowLevel.PeriodicFrame.kStatus2, (int)(1000 / ODOMETRY_FREQUENCY)),
          () -> true,
          Constants.MiscConstants.CONFIGURATION_ATTEMPTS
          );

  faultInitializing |= RaiderUtils.applyAndCheckRev(
          transportMotor::burnFlashWithDelay, () -> true, Constants.MiscConstants.CONFIGURATION_ATTEMPTS
  );


  transportAlert.set(faultInitializing);
}
