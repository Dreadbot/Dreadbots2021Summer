package frc.robot.subsystem;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.utility.Constants;

public class Feeder {
	// Hardware
	private CANSparkMax genevaDrive = new CANSparkMax(Constants.GENEVA_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
	private Solenoid punch = new Solenoid(Constants.PUNCH_SOLENOID_ID);

	// Limit Switch Sensors
	private DigitalInput genevaLimitSwitch = new DigitalInput(Constants.GENEVA_LIMIT_SWITCH_PORT);
	private DigitalInput punchLimitSwitch = new DigitalInput(Constants.PUNCH_LIMIT_SWITCH_PORT);

	public Feeder() {
		genevaDrive.setIdleMode(CANSparkMax.IdleMode.kBrake);
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