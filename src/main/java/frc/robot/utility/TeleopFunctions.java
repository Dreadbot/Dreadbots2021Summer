package frc.robot.utility;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystem.Manipulator;
import frc.robot.subsystem.SparkDrive;

public class TeleopFunctions {
	private final double slop = 1.5;
	private final int timeToAdjust = 10;
	private DreadbotController primaryJoystick;
	private Manipulator manipulator;
	private SparkDrive sparkDrive;
	private int shooterButton = 1;
	private PIDController pidController;
	private double p;
	private double i;
	private double d;
	private double currentRotationRate, proportion, minimumRotationSpeed = 0;
	private int turnButtonTimeout;
	private boolean turnComplete = true;

	public TeleopFunctions(DreadbotController primaryJoystick, Manipulator manipulator, SparkDrive sparkDrive) {
		this.primaryJoystick = primaryJoystick;
		this.manipulator = manipulator;
		this.sparkDrive = sparkDrive;

		p = 0.1; //Changed for the carpet at the church
		i = 0.0;
		d = 0.00135;

		SmartDashboard.putNumber("Turn P Value", p);
		SmartDashboard.putNumber("Turn I Value", i);
		SmartDashboard.putNumber("Turn D Value", d);

		pidController = new PIDController(p, i, d, .02);

		pidController.setSetpoint(0.0);
	}

	public void turnToAngle(double targetAngle, double proportion) {
		targetAngle += SmartDashboard.getNumber("Turn Fudge Factor", 0.0);
		// If a button is pressed, reset the counter, and signal that a turn is
		// initiiated
		minimumRotationSpeed = SmartDashboard.getNumber("Min Rot Speed", 0.15);
		// SmartDashboard.PutBoolean("turn complete?", turn_complete);
		// Find the difference between the current angle and the target angle, multiply
		// by a set value, and use that to find the rate
		double error = (double) sparkDrive.getGyroscope().getYaw() - targetAngle;
		System.out.println("Error: " + error);
		currentRotationRate = error * proportion;
		System.out.println(" 1: " + currentRotationRate);

		// Set the lower bound of the rotation speed so it is not less than the power
		// necessary to turn the robot
		if (currentRotationRate > 0)
			currentRotationRate += minimumRotationSpeed;
		else if (currentRotationRate < 0)
			currentRotationRate -= minimumRotationSpeed;
		System.out.println(" 2: " + currentRotationRate);
		// Set the upper bound of the rotation rate
		currentRotationRate = (currentRotationRate > 1) ? 1 : currentRotationRate;
		currentRotationRate = (currentRotationRate < -1) ? -1 : currentRotationRate;
		System.out.println(" 3: " + currentRotationRate);

		// if we are not within the slop, then we are not done with the turn
		if (Math.abs(error) > slop) {
			turnComplete = false;
			turnButtonTimeout = 0;
		}

		// If the turn has made it within the allowable error constant, increment the
		// count
		if (Math.abs(error) < slop) {
			turnButtonTimeout++;
		}

		SmartDashboard.putNumber("Error", error);
		SmartDashboard.putNumber("Current Rotation Rate", currentRotationRate);
		SmartDashboard.putNumber("Target Angle", targetAngle);

		// Drive the robot using the SparkDrive::TankDrive function, with the
		// forward/backward axis still based on
		// controller input, but the rotation axis of the drive base based on the
		// rotation rate found
		// TODO Write this better
		SparkDrive.DriveMode driveMode = SparkDrive.DriveMode.NORMAL;
		driveMode = primaryJoystick.isRightTriggerPressed() ? SparkDrive.DriveMode.TURBO : SparkDrive.DriveMode.NORMAL;
		driveMode = primaryJoystick.isRightBumperPressed() ? SparkDrive.DriveMode.TURTLE : SparkDrive.DriveMode.NORMAL;

		sparkDrive.tankDrive(primaryJoystick.getYAxis(), -currentRotationRate, driveMode);

		// If the difference between the current angle and the target angle is within an
		// allowable constant,
		// and enough time has elapsed in while within that bound to allow for the
		// turning to settle,
		// declare the turn finished and reset the gyro
		if (Math.abs(error) < slop && turnButtonTimeout > timeToAdjust) {
			turnComplete = true;
			sparkDrive.stop();
		}
	}

	public void WPITurnToAngle(double targetAngle) {
		// If a button is pressed, reset the counter, and signal that a turn is
		// initiiated
		// frc::SmartDashboard::PutBoolean("turn complete?", turn_complete);
		// Find the difference between the current angle and the target angle, multiply
		// by a set value, and use that to find the rate
		// System.out.println("Gyroscope Angle: " 	+sparkDrive.getGyroscope().getYaw());
		double error = (((double) sparkDrive.getGyroscope().getYaw()) - targetAngle) * -1;
		System.out.println("Yaw: " + sparkDrive.getGyroscope().getYaw() + " ERROR: " + error);

		// std::cout << "Error: " << error;

		p = SmartDashboard.getNumber("Turn P Value", 0.002);
		i = SmartDashboard.getNumber("Turn I Value", 0.019);
		d = SmartDashboard.getNumber("Turn D Value", 0.000001);
		updatePIDController();


		currentRotationRate = pidController.calculate(error);

		// Set the upper bound of the rotation rate
		DreadbotMath.clampValue(currentRotationRate, -1.0, 1.0);

		SmartDashboard.putNumber("Error", error);
		SmartDashboard.putNumber("Gyro Angle", sparkDrive.getGyroscope().getYaw());
		SmartDashboard.putNumber("Current Rotation Rate", currentRotationRate);
		SmartDashboard.putNumber("Target Angle", targetAngle);

		// Drive the robot using the SparkDrive::TankDrive function, with the
		// forward/backward axis still based on
		// controller input, but the rotation axis of the drive base based on the
		// rotation rate found
		SparkDrive.DriveMode driveMode = SparkDrive.DriveMode.NORMAL;
		/*driveMode = primaryJoystick.isRightTriggerPressed() ? SparkDrive.DriveMode.TURBO : SparkDrive.DriveMode.NORMAL;
		driveMode = primaryJoystick.isRightBumperPressed() ? SparkDrive.DriveMode.TURTLE : SparkDrive.DriveMode.NORMAL;*/
		driveMode = SparkDrive.DriveMode.TURTLE;
		sparkDrive.tankDrive(0, -currentRotationRate, driveMode);

		// If the difference between the current angle and the target angle is within an
		// allowable constant,
		// and enough time has elapsed in while within that bound to allow for the
		// turning to settle,
		// declare the turn finished and reset the gyro

		// if (Math.abs(error) < slop && turnButtonTimeout > timeToAdjust) {
		// 	turnComplete = true;
		// 	// m_sparkDrive->GetGyroscope()->ZeroYaw();
		// 	sparkDrive.stop();
		// }

		if (Math.abs(error) < slop && Math.abs(sparkDrive.getWheelSpeeds().leftMetersPerSecond) < 1) {
			turnComplete = true;
			// m_sparkDrive->GetGyroscope()->ZeroYaw();
			sparkDrive.stop();
		}
		else{
			turnComplete = false;
		}
	}

	public double calculateTurnToAngle(double targetAngle) {
		// If a button is pressed, reset the counter, and signal that a turn is
		// initiiated
		// SmartDashboard.PutBoolean("turn complete?", turn_complete);
		// Find the difference between the current angle and the target angle, multiply
		// by a set value, and use that to find the rate
		double error = ((double) sparkDrive.getGyroscope().getYaw()) - targetAngle;
		// std::cout << "Error: " << error;

		updatePIDController();

		currentRotationRate = pidController.calculate(error);

		// Set the upper bound of the rotation rate
		DreadbotMath.clampValue(currentRotationRate, -1.0, 1.0);

		SmartDashboard.putNumber("Error", error);
		SmartDashboard.putNumber("Current Rotation Rate", currentRotationRate);
		SmartDashboard.putNumber("Target Angle", targetAngle);

		// Drive the robot using the SparkDrive::TankDrive function, with the
		// forward/backward axis still based on
		// controller input, but the rotation axis of the drive base based on the
		// rotation rate found

		// If the difference between the current angle and the target angle is within an
		// allowable constant,
		// and enough time has elapsed in while within that bound to allow for the
		// turning to settle,
		// declare the turn finished and reset the gyro
		if (Math.abs(error) < slop && turnButtonTimeout > timeToAdjust) {
			turnComplete = true;
			// m_sparkDrive->GetGyroscope()->ZeroYaw();
			currentRotationRate = 0;
		}

		return currentRotationRate;
	}

	public void updatePIDController() {
		// SmartDashboard.putNumber("Turn P Value", p);
		// SmartDashboard.putNumber("Turn I Value", i);
		// SmartDashboard.putNumber("Turn D Value", d);

		pidController.setPID(p, i, d);
	}

	public boolean getTurnStatus() {
		return turnComplete;
	}

	public void setTurnStatus(boolean turnComplete) {
		this.turnComplete = turnComplete;
	}

	public void shooterFunction() {
		if (primaryJoystick.getRawButton(shooterButton)) {
			manipulator.getShooter().shoot(1000);
		} else {
			manipulator.getShooter().shoot(0);
		}
	}
}
