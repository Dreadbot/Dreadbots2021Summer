package frc.robot.subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Solenoid;

public class Climber {

    private Solenoid telescopeSol;
    private CANSparkMax winchMotor;
    private CANPIDController winchPID;

    public Climber() {
        winchMotor = new CANSparkMax(-1, CANSparkMaxLowLevel.MotorType.kBrushless);
        telescopeSol = new Solenoid(-1);
    }

    public void SetTelescope(boolean extended) {
        telescopeSol.set(extended);
    }

    public void SetWinch(double winchSpeed) {
        winchMotor.set(winchSpeed);
    }

}