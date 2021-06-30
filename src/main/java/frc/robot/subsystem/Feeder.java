package frc.robot.subsystem;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.utility.DreadbotConstants;

public class Feeder {
    // Hardware
    private CANSparkMax genevaDrive = new CANSparkMax(DreadbotConstants.GENEVA_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    private CANEncoder genevaEncoder;
    private CANPIDController genevaController;
    private Solenoid punch = new Solenoid(DreadbotConstants.PUNCH_SOLENOID_ID);

    // Limit Switch Sensors
    private DigitalInput genevaLimitSwitch = new DigitalInput(DreadbotConstants.GENEVA_LIMIT_SWITCH_PORT);
    private DigitalInput punchLimitSwitch = new DigitalInput(DreadbotConstants.PUNCH_LIMIT_SWITCH_PORT);

    public Feeder() {
        genevaDrive.setIdleMode(CANSparkMax.IdleMode.kBrake);
        genevaController = genevaDrive.getPIDController();
        genevaEncoder = genevaDrive.getEncoder();

        genevaEncoder.setPosition(0);

        genevaController.setP(0.002);
        genevaController.setI(1e-6);
        genevaController.setD(.02);
        genevaController.setFF(0.000015);
        genevaController.setIZone(0);
        genevaController.setOutputRange(-1.0, 1.0);
    }

    public void setSpin(double power) {
        genevaDrive.set(power);
    }

    public boolean getPunchExtension() {
        return punch.get();
    }

    public void setPunchExtension(boolean extended) {
        punch.set(extended);
    }

    public boolean getGenevaSwitchState() {
        return !genevaLimitSwitch.get();
    }

    public boolean getPunchSwitchState() {
        return punchLimitSwitch.get();
    }

    public double getGenevaPosition() {
        return genevaEncoder.getPosition();
    }
}
