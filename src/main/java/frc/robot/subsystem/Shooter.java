package frc.robot.subsystem;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpiutil.math.MathUtil;

public class Shooter {
	// Limit Switch Variables
	int minHoodPosition;
	int maxHoodPosition;
	boolean readyToAim = false;
	boolean lowerLimitHit = false;
	boolean upperLimitHit = false;

	// Motor Controllers
	private final CANSparkMax shooterMotor;
	private final CANSparkMax aimingMotor;

	// PID Controllers
	private final CANPIDController shooterMotorPID;
	private final CANPIDController aimingMotorPID;

	// Encoders
	private final CANEncoder shooterMotorEncoder;
	private final CANEncoder aimingMotorEncoder;

	// Limit Switches
	private final DigitalInput upperLimitSwitch;
	private final DigitalInput lowerLimitSwitch;

	// Vision Light
	private final Solenoid visionLEDRing;

	// Shooting Mechanism Variables
	private final double speed;
	private double aimPosition;
	private double range;

	public Shooter() {
		// Instantiate Motor Controllers
		shooterMotor = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);
		aimingMotor = new CANSparkMax(8, CANSparkMaxLowLevel.MotorType.kBrushless);
		shooterMotor.restoreFactoryDefaults();
		// Get the PID Controller Objects
		shooterMotorPID = shooterMotor.getPIDController();
		aimingMotorPID = aimingMotor.getPIDController();

		// Get the Encoder Objects
		shooterMotorEncoder = shooterMotor.getEncoder();
		aimingMotorEncoder = aimingMotor.getEncoder();

		// Instantiate Limit Switches
		upperLimitSwitch = new DigitalInput(1);
		lowerLimitSwitch = new DigitalInput(2);

		// Instantiate Vision LED Ring
		visionLEDRing = new Solenoid(7);

		// Configure PID Controllers (values are tuned)
		// shooterMotorPID.setP(0.0025);
		// shooterMotorPID.setI(0.00000033);
		// shooterMotorPID.setD(0.03);

		shooterMotorPID.setP(9e-3);
		shooterMotorPID.setI(1e-6);
		shooterMotorPID.setD(0);
		shooterMotorPID.setIZone(0);
		shooterMotorPID.setFF(0.000015);
		shooterMotorPID.setOutputRange(-1.0, 1.0);
		speed = 0.0;

		aimingMotorPID.setP(0.1);
		aimingMotorPID.setI(0);
		aimingMotorPID.setD(0);
		aimingMotorPID.setIZone(0);
		aimingMotorPID.setFF(0.000015);
		aimingMotorPID.setOutputRange(-1.0, 1.0);

		// aimingMotor.set(0.25);
	}

	public void shoot(double rpm) {
		shooterMotorPID.setReference(rpm, ControlType.kVelocity);
	}

	public void aimHeight(double rotations) {
		aimingMotorPID.setReference(rotations, ControlType.kPosition);
	}

	public void setAimHeightP(double p) {
		aimingMotorPID.setP(p);
	}

	public void setShootingPercentOutput(double percentOutput) {
		shooterMotor.set(percentOutput);
	}

	public void setHoodPercentOutput(double percentOutput) {
		System.out.println(!lowerLimitSwitch.get());
		System.out.println(!upperLimitSwitch.get());
		if(getLowerLimitSwitch() && percentOutput > 0){
			percentOutput = 0.0;
		}
		else if(getUpperLimitSwitch() && percentOutput <0){
			percentOutput = 0.0;
		}

		aimingMotor.set(percentOutput);
	}

	public int getShootingSpeed() {
		// Cast to int for ease of comparison
		return (int) shooterMotorEncoder.getVelocity();
	}

	public void setLowerLimit(int position) {
		minHoodPosition = position;
		lowerLimitHit = true;
	}

	public void setUpperLimit(int position) {
		maxHoodPosition = position;
		upperLimitHit = true;
	}

	public void setUpperBool(boolean value) {
		upperLimitHit = value;
	}

	public void setLowerBool(boolean value) {
		lowerLimitHit = value;
	}

	boolean getAimReadiness() {
		return readyToAim;
	}

	public void setAimReadiness(boolean ready) {
		readyToAim = ready;
		if (ready)
			range = maxHoodPosition - minHoodPosition;
	}

	public int getHoodPosition() {
		return (int) aimingMotorEncoder.getPosition();
	}

	public void setHoodPosition(double position) {
		position = MathUtil.clamp(position, -0.12d, 1.0d);

		position = minHoodPosition + (position * range);

		aimingMotorPID.setReference(position, ControlType.kPosition);
	}

	public boolean getUpperLimitSwitch() {
		return !upperLimitSwitch.get();
	}

	public boolean getLowerLimitSwitch() {
		return !lowerLimitSwitch.get();
	}

	public boolean getUpperLimitBool() {
		return upperLimitHit;
	}

	public boolean getLowerLimitBool() {
		return lowerLimitHit;
	}

	public void hoodCalibration(){
		if(getLowerLimitSwitch() && !getLowerLimitBool()){ 
		  System.out.println("***************LOWER LIMIT TRIGGERED");
		  setLowerLimit(getHoodPosition());
		  setHoodPercentOutput(-0.75);
		}
		else if(getUpperLimitSwitch() && !getUpperLimitBool()){
		  System.out.println("***************UPPER LIMIT TRIGGERED");
		  setUpperLimit(getHoodPosition());
		  setHoodPercentOutput(0.75);
		}
		else if (getUpperLimitBool() && getLowerLimitBool() && !getAimReadiness()){
		  setAimReadiness(true);
		  setHoodPosition(0.5);
		}
		if(getAimReadiness()){
		//   position = frc::SmartDashboard::GetNumber("Hood Position", 0.5);

		  //std::cout << "Adjusting Position to: " << position << std::endl;
		}
	}
	public void setPID(double p, double i, double d) {
		shooterMotorPID.setP(p);
		shooterMotorPID.setI(i);
		shooterMotorPID.setD(d);
	}

	public void setVisionLight(boolean visionLightEnabled) {
		visionLEDRing.set(visionLightEnabled);
	}
}