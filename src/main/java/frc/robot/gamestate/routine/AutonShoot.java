package frc.robot.gamestate.routine;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Main;
import frc.robot.gamestate.Teleoperated;
import frc.robot.subsystem.Manipulator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonShoot extends AutonSegment{
	int numBalls;
	double targetAngle;
	double targetDistance;
	Teleoperated teleoperated;
	Manipulator manipulator;
	boolean doneShooting;

	public AutonShoot(Teleoperated teleoperated, Manipulator manipulator, int numBalls){
		this.teleoperated = teleoperated;
		this.manipulator = manipulator;
		this.numBalls = numBalls;
	}

	@Override
	public void autonomousInit() {
		this.complete = false;
		doneShooting = false;
		teleoperated.setAimShootState(Teleoperated.AimShootStates.AIMING);
		manipulator.resetManipulatorElements();
		targetAngle = SmartDashboard.getNumber("selectedAngle", 0.0);
		targetDistance = SmartDashboard.getNumber("selectedDistance", 0.0);
	}

	@Override
	public void autonomousPeriodic() {
		manipulator.getShooter().setPID(SmartDashboard.getNumber("Shooter P", .0025),
			SmartDashboard.getNumber("Shooter I", 3.3e-7),
			SmartDashboard.getNumber("Shooter D", 0.03));
		SmartDashboard.putNumber("SHOOTER STATE", teleoperated.getAimShootStates().ordinal());
		if(!doneShooting){
			if(teleoperated.aimingContinuousShoot(targetDistance, targetAngle, 0.4) >= numBalls){
				//this.doneShooting = true;
				manipulator.resetManipulatorElements();
				//manipulator.sensorAdvanceGeneva(true, true);
				this.complete = true;
			}
		}

		else{
			manipulator.sensorAdvanceGeneva(false, true);
		}
	}
	
}