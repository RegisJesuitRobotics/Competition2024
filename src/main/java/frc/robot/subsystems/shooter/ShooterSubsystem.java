package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.utils.Alert;

import static frc.robot.Constants.ShooterConstants.*;


public class ShooterSubsystem extends SubsystemBase {
    //TODO I have no clue what the final cad looks like so these are all arbitrary

    private final TelemetryCANSparkMax topFly = new TelemetryCANSparkMax(TOP_SHOOTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless, "/shooter/top", true);
    private final TelemetryCANSparkMax bottomFly = new TelemetryCANSparkMax(BOTTOM_SHOOTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless, "/shooter/bottom", true);

    private final TelemetryCANSparkMax topTransport = new TelemetryCANSparkMax(TOP_TRANSPORT_ID, CANSparkMaxLowLevel.MotorType.kBrushless, "/shooterTransport/top", true);
    private final RelativeEncoder topFlyEncoder = topFly.getEncoder();
    private final RelativeEncoder bottomFlyEncoder = bottomFly.getEncoder();
    private final SimpleMotorFeedforward FF = new SimpleMotorFeedforward(SHOOTER_FF_GAINS.sFF.get(), SHOOTER_FF_GAINS.sFF.get(), SHOOTER_FF_GAINS.vFF.get());

    private final DoubleTelemetryEntry topFlyVoltageReq = new DoubleTelemetryEntry("/shooter/topVoltage", false);
    private final DoubleTelemetryEntry bottomFlyVoltageReq = new DoubleTelemetryEntry("/shooter/bottomVoltage", false);

    private final DoubleTelemetryEntry topTransportVoltageReq = new DoubleTelemetryEntry("/shooterTransport/topVoltage", false);
    private final DoubleTelemetryEntry bottomTransportVoltageReq = new DoubleTelemetryEntry("/shooterTransport/bottomVoltage", false);

    private final DigitalInput shooterFrisbeeSensor = new DigitalInput(SHOOTER_SENSOR);



    private double topVoltage = 0;
    private double bottomVoltage = 0;

    public  void setBothFlyVoltage(double voltage){
        topFly.setVoltage(voltage);
        bottomFly.setVoltage(voltage);
    }
    public void setTopVoltage(double voltage){
        topFly.setVoltage(voltage);
    }

    public void setBottomVoltage(double voltage){
        bottomFly.setVoltage(voltage);
    }

    public void setRPM(int rpm){
       double forwardVol = FF.calculate(rpm);

        setBothFlyVoltage(forwardVol);

    }

    public void runShooterTransportIn(){
        topTransport.setVoltage(TRANSPORT_VOLTAGE);
    }

    public void shooterTransportStop(){
        topTransport.setVoltage(0);
    }
    public void runShooterTransportOut(){
        topTransport.setVoltage(-TRANSPORT_VOLTAGE);
    }
    public void TransportStop(){
        topTransport.setVoltage(0);
    }

    public boolean isAtSensor(){
        if (shooterFrisbeeSensor.get()){
            return true;
        }
        else{
            return false;
        }
    }


  public double getTopEncoderVelocity(){
        return topFlyEncoder.getVelocity();
  }
  public double getBottomEncoderVelocity(){
        return bottomFlyEncoder.getVelocity();
  }

}