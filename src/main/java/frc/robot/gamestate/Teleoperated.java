package frc.robot.gamestate;

import javax.security.auth.x500.X500Principal;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystem.Manipulator;
import frc.robot.subsystem.SparkDrive;
import frc.robot.subsystem.SparkDrive.DriveMode;
import frc.robot.utility.Constants;
import frc.robot.utility.DreadbotController;
import frc.robot.utility.TeleopFunctions;

public class Teleoperated {
	private DreadbotController primaryJoystick;
	private DreadbotController secondaryJoystick;
	private Manipulator manipulator;
	private SparkDrive sparkDrive;
	private TeleopFunctions teleopFunctions;
	private AimShootStates aimShootState;

	final int maxAimCounts;
	int aimCounts;

	double selectedAngle = 0;
	int lastCount = 0;
	int staleCount = 0;

	double distance = 120;
	double rotSpeed = 0;

	boolean firstAim = true;

	public Teleoperated(DreadbotController primaryJoystick,
	                    DreadbotController secondaryJoystick,
	                    Manipulator manipulator,
	                    SparkDrive sparkDrive) {
		this.primaryJoystick = primaryJoystick;
		this.secondaryJoystick = secondaryJoystick;

		this.manipulator = manipulator;
		this.sparkDrive = sparkDrive;
		this.teleopFunctions = new TeleopFunctions(secondaryJoystick, manipulator, sparkDrive);

		aimShootState = AimShootStates.AIMING;
		maxAimCounts = 150;
		aimCounts = 0;
	}

	public void initIntake() {
		manipulator.getIntake().deployIntake();
	}

	public void initDrive() {
		sparkDrive.getGyroscope().zeroYaw();
	}

	public void teleopIntake() {
		if (secondaryJoystick.isXButtonPressed()) {
			manipulator.getIntake().setSpeed(-4000);
		} else if (secondaryJoystick.isAButtonPressed()) {
			manipulator.getIntake().setSpeed(4000);
		} else {
			manipulator.getIntake().setPercentOutput(0);
		}
	}

	public void teleopDrive() {
		//TODO Write this better
		DriveMode driveMode = DriveMode.NORMAL;
		driveMode = primaryJoystick.isRightTriggerPressed() ? DriveMode.TURBO : DriveMode.NORMAL;
		driveMode = primaryJoystick.isRightBumperPressed() ? DriveMode.TURTLE : DriveMode.NORMAL;

		sparkDrive.tankDrive(primaryJoystick.getYAxis(),
			primaryJoystick.getZAxis(),
			driveMode);
	}

	public void teleopShooter() {
		SmartDashboard.putNumber("Current Angle", sparkDrive.getGyroscope().getYaw());
		double hood_position = SmartDashboard.getNumber("Hood Position", 0.5);

		if (SmartDashboard.getNumber("detectionCount", lastCount) == lastCount)
			staleCount++;
		else
			staleCount = 0;

		if (aimShootState == AimShootStates.AIMING) {
			distance = SmartDashboard.getNumber("selectedDistance", 120);
		}
		//update the latest count, for use on next loop iteration
		lastCount = (int) SmartDashboard.getNumber("detectionCount", lastCount);

		//if we are done turning (not currently turning), then update angle from vision
		if (teleopFunctions.getTurnStatus() && firstAim) {
			selectedAngle = SmartDashboard.getNumber("selectedAngle", 0.0);
			// selectedAngle = sparkDrive.getGyroscope().getYaw() - selectedAngle;
			firstAim = false;
		}
		//Only turn and shoot when we hold the button, and we have seen the target recently
		if (secondaryJoystick.isYButtonPressed()) {
			// double shooting_hood_position = SmartDashboard.getNumber("Hood Position", 0.5);
			double shooting_rpm = SmartDashboard.getNumber("Shooter Target Speed", 4000);
			System.out.println("Cont Shooting");
			// double rpm = manipulator.getSelectedRPM(distance);
			manipulator.continuousShoot(-0.12, 0.4, shooting_rpm);
			SmartDashboard.putNumber("camNumber", 0);
		} else if (secondaryJoystick.isBButtonPressed() && staleCount < 5) {
			//System.out.println("B BUTTON PRESSED");
			aimingContinuousShoot(distance, selectedAngle, 0.4);
			SmartDashboard.putNumber("camNumber", 0);
			staleCount = 0;
		} else if (secondaryJoystick.isBButtonPressed()) {
			SmartDashboard.putNumber("camNumber", 0);
		} else if (secondaryJoystick.isRightBumperPressed()) {
			manipulator.sensorAdvanceGeneva(true, true);
		} else if (secondaryJoystick.isLeftBumperPressed()) {
			manipulator.sensorAdvanceGeneva(true, false);
		} else if (manipulator.getSensorAdvanceGenevaState() == 2) { //2 means Geneva is stopped
			//std::cout << "Reseting" << std::endl;
			manipulator.resetManipulatorElements();
			teleopFunctions.setTurnStatus(true);
			aimCounts = 0;
			aimShootState = AimShootStates.AIMING;
			rotSpeed = 0;
			sparkDrive.getGyroscope().reset();
			firstAim = true;
		} else {
			// SmartDashboard.putNumber("camNumber", 1);
			manipulator.resetManipulatorElements();
		}
	}

	public int aimingContinuousShoot(double distance, double targetAngle, double genevaSpeed) {
		int numPunches = 0;
//		double rpm = manipulator.getSelectedRPM(distance);
		double rpm = SmartDashboard.getNumber("tuning RPM", 3500);
		SmartDashboard.putNumber("Target Shooting Velocity", rpm);
//		double hoodPosition = manipulator.getSelectedHoodPosition(distance);
		double hoodPosition = SmartDashboard.getNumber("tuning Hood Position", 0.5);

		aimShootState = (aimCounts < maxAimCounts) ? AimShootStates.AIMING : AimShootStates.SHOOTING;
		SmartDashboard.putNumber("aimShootState", aimShootState.ordinal());
		switch (aimShootState) {
			case AIMING:
				// rotSpeed = teleopFunctions.calculateTurnToAngle(targetAngle);
				teleopFunctions.WPITurnToAngle(targetAngle);
				manipulator.prepareShot(-rpm, hoodPosition);
				break;
			case SHOOTING:
				sparkDrive.stop();
				numPunches = manipulator.continuousShoot(hoodPosition, genevaSpeed, rpm);
				break;
		}
		aimCounts++;
		return numPunches;
	}

	// public void aimingContinuousShoot(double rpm, double hoodPosition, double targetAngle, double genevaSpeed) {
	// 	SmartDashboard.putNumber("aim counts", aimCounts);

	// 	aimShootState = (aimCounts < maxAimCounts) ? AimShootStates.AIMING : AimShootStates.SHOOTING;

	// 	switch (aimShootState) {
	// 		case AIMING:
	// 			teleopFunctions.WPITurnToAngle(targetAngle);
	// 			manipulator.prepareShot(rpm, hoodPosition);
	// 			break;
	// 		case SHOOTING:
	// 			sparkDrive.stop();
	// 			manipulator.continuousShoot(hoodPosition, genevaSpeed, rpm);
	// 			break;
	// 	}
	// 	aimCounts++;
	// }

	public void resetAimCounts() {
		aimCounts = 0;
	}

    public TeleopFunctions getTeleopFunctions() {
		return teleopFunctions;
    }

    public enum AimShootStates {
		AIMING,
		SHOOTING;
	}

	public void setAimShootState(AimShootStates a){
		aimShootState = a;
	}
	public AimShootStates getAimShootStates(){
		return aimShootState;
	}
}
