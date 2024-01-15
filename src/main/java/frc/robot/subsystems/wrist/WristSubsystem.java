package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;

import static frc.robot.Constants.WristConstants.*;

public class WristSubsystem extends SubsystemBase {
    
    private final DigitalInput atBottom = new DigitalInput(WRIST_SWITCH_ID);
    private final TelemetryCANSparkMax wristMotor = new TelemetryCANSparkMax(WRIST_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless, "/wrist/motors", true);

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(WRIST_FF_GAINS.aFF.get(), WRIST_FF_GAINS.sFF.get(), WRIST_FF_GAINS.vFF.get());
    private final TunableTelemetryProfiledPIDController controller = new TunableTelemetryProfiledPIDController("wrist/pid", WRIST_PID_GAINS, TRAPEZOIDAL_PROFILE_GAINS);
    private final RelativeEncoder relativeEncoder = wristMotor.getEncoder();
    
    public WristSubsystem(){
        
    }
    
    public boolean atTransportAngle(){
        return atBottom.get();
    }
    
    public double getPosition(){
        return relativeEncoder.getPosition();
    }
    
    public void setDesiredPosition(double desiredPosition){

        controller.setGoal(desiredPosition);
    }

    public void setVoltage(double voltage){
        wristMotor.setVoltage(voltage);
    }

    @Override
    public void periodic(){
        double feedbackOutput = controller.calculate(getPosition());

        TrapezoidProfile.State currentSetpoint = controller.getSetpoint();
        double combinedOutput = feedbackOutput + feedforward.calculate(currentSetpoint.velocity);

        setVoltage(combinedOutput);


    }
}
