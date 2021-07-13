package frc.robot.gamestate.routine;

import frc.robot.subsystem.SparkDrive;

/**
 * Auton segment for driving forward for a given distance in meters.
 */
public class AutonDrive extends AutonSegment {
	private static final double kP = 1;

	private final double distanceMeters;
	private double initialPositionMeters;

	private boolean forward = true;

	private final SparkDrive sparkDrive;

	/**
	 * @param distanceMeters The given distance to drive, in meters.
	 * @param sparkDrive The drive object reference
	 */
	public AutonDrive(double distanceMeters, SparkDrive sparkDrive) {
		this.distanceMeters = distanceMeters;
		this.sparkDrive = sparkDrive;
	}

	/**
	 * @param distanceMeters The given distance to drive, in meters.
	 * @param forward Determines if the motion is moving forward (according to motor direction) or not.
	 * @param sparkDrive The drive object reference
	 */
	public AutonDrive(double distanceMeters, boolean forward, SparkDrive sparkDrive) {
		this.distanceMeters = distanceMeters;
		this.forward = forward;
		this.sparkDrive = sparkDrive;
	}

	@Override
	public void autonomousInit() {
		initialPositionMeters = sparkDrive.getAverageEncoderDistance();
	}

	@Override
	public void autonomousPeriodic() {
		double distanceFromInitialPosition = Math.abs(sparkDrive.getAverageEncoderDistance() - initialPositionMeters);

		if(distanceFromInitialPosition < distanceMeters) {
			sparkDrive.drive((forward ? 1.0 : -1.0) * 0.5, 0.0);
			return;
		}

		sparkDrive.drive(0.0, 0.0);
		complete = true;
	}

	@Override
	public void disabledInit() {
		super.disabledInit();
	}
}
