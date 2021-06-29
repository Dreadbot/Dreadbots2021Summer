package frc.robot.gamestate.routine;

import frc.robot.subsystem.SparkDrive;
import frc.robot.utility.TeleopFunctions;

public class AutonTraditional extends AutonSegment {
	private AutonDrive driveSegment;
	private RotateToAngle rotateSegment;

	public AutonTraditional(double driveLength, int angle, SparkDrive sparkDrive, TeleopFunctions teleopFunctions) {
		driveSegment = new AutonDrive(driveLength, sparkDrive);
		rotateSegment = new RotateToAngle(angle, sparkDrive, teleopFunctions);
	}

	@Override
	public void autonomousInit() { }

	@Override
	public void autonomousPeriodic() {
		if(!driveSegment.isComplete()) {
			driveSegment.autonomousPeriodic();
			return;
		}

		if(!rotateSegment.isComplete()) {
			rotateSegment.autonomousPeriodic();
			return;
		}

		this.complete = true;
	}
}
