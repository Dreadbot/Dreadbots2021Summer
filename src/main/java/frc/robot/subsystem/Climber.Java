import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj.Solenoid;

public class Climber{
    
    private Solenoid telescopeSol;
    private CANSparkMax winchMotor;
    private CANPIDController winchPID;

    public Climber() {
        winchMotor = new CANSparkMax(-1, CANSparkMaxLowLevel.MotorType.kBrushless);
        telescopeSol = new Solenoid(-1);
    }

    public void SetTelescope(bool extended){
        telescopeSol.set(extended);
    }
    public void SetWinch(double winchSpeed){
        winchMotor.set(winchSpeed);
    }

}