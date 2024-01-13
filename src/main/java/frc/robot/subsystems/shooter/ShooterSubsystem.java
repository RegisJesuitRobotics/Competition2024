package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import static frc.robot.Constants.ShooterConstants.*;


public class ShooterSubsystem extends SubsystemBase {
    //TODO I have no clue what the final cad looks like so these are all arbitrary

    private final TelemetryCANSparkMax topFly = new TelemetryCANSparkMax(TOP_SHOOTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless, "/shooter/top", true);
    private final TelemetryCANSparkMax bottomFly = new TelemetryCANSparkMax(BOTTOM_SHOOTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless, "/shooter/bottom", true);

    private final RelativeEncoder topFlyEncoder = topFly.getEncoder();
    private final RelativeEncoder bottomFlyEncoder = bottomFly.getEncoder();
    private final SimpleMotorFeedforward FF = new SimpleMotorFeedforward(SHOOTER_FF_GAINS.sFF.get(), SHOOTER_FF_GAINS.sFF.get(), SHOOTER_FF_GAINS.vFF.get());

    private final DoubleTelemetryEntry topFlyVoltageReq = new DoubleTelemetryEntry("/shooter/topVoltage", false);
    private final DoubleTelemetryEntry bottomFlyVoltageReq = new DoubleTelemetryEntry("/shooter/bottomVoltage", false);

    private double topVoltage = 0;
    private double bottomVoltage = 0;

    public  void setBothVoltage(double voltage){
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

    setBothVoltage(forwardVol);

    }

  public double getTopEncoderVelocity(){
        return topFlyEncoder.getVelocity();
  }
  public double getBottomEncoderVelocity(){
        return bottomFlyEncoder.getVelocity();
  }

}