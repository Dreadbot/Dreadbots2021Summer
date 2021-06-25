package frc.robot.subsystem;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.utility.Constants;

public class Feeder {
	private final int genevaSwitchPort = 9;
	private final int punchSwitchPort = 3;
	private CANSparkMax genevaDrive;
	private CANPIDController genevaControler;
	private CANEncoder genevaEncoder;
	private Solenoid punch;
	private DigitalInput genevaLimitSwitch;
	private DigitalInput punchLimitSwitch;

	public Feeder() {
		this.genevaDrive = new CANSparkMax(Constants.GENEVA_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
		this.punch = new Solenoid(Constants.PUNCH_SOLENOID_ID);

		genevaDrive.setIdleMode(CANSparkMax.IdleMode.kBrake);
		genevaControler = genevaDrive.getPIDController();
		genevaEncoder = genevaDrive.getEncoder();
		genevaEncoder.setPosition(0);
		genevaLimitSwitch = new DigitalInput(genevaSwitchPort);
		punchLimitSwitch = new DigitalInput(punchSwitchPort);

		genevaControler.setP(.002);
		genevaControler.setI(1e-6);
		genevaControler.setD(.02);
		genevaControler.setFF(.000015);
		genevaControler.setIZone(0);
		genevaControler.setOutputRange(-1.0, 1.0);
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

	public void ExtendRetract(int millisecondsBetween) {
		setPunchExtension(true);
		try {
			Thread.sleep(millisecondsBetween);
		} catch (InterruptedException ex) {
			System.err.println(ex);
		}
		setPunchExtension(false);
	}
}