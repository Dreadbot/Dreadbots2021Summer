package frc.robot.gamestate.routine;

import frc.robot.subsystem.SparkDrive;

public class AutonDrive extends AutonSegment {
	private static final double kP = 1;

	private final double distanceMeters;
	private double initialPositionMeters;

	private final SparkDrive sparkDrive;

	public AutonDrive(double distanceMeters, SparkDrive sparkDrive) {
		this.distanceMeters = distanceMeters;
		this.sparkDrive = sparkDrive;
	}

	@Override
	public void autonomousInit() {
		initialPositionMeters = sparkDrive.getAverageEncoderDistance();
	}

	@Override
	public void autonomousPeriodic() {
		if(Math.abs(sparkDrive.getAverageEncoderDistance()) - initialPositionMeters < distanceMeters) {
			sparkDrive.tankDrive(0.5, 0.5, SparkDrive.DriveMode.NORMAL);
		}  else {
			sparkDrive.tankDrive(0.0, 0.0);
			complete = true;
		}
	}

	@Override
	public void disabledInit() {
		super.disabledInit();
	}
}
