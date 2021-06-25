import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj.Solenoid;

public class Climber{
    private Solenoid telescopeSol;
    private CANSparkMax winchMotor;
    private CANPIDController winchPID;

    public void SetTelescope(bool extended){
        telescopeSol.set(extended);
    }
    public void SetWinch(double winchSpeed){
        winchMotor.set(winchSpeed);
    }

}