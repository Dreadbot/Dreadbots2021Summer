package frc.robot.gamestate.routine;

/**
 * Base class for all Autonomous Segments in an Autonomous Routine.
 * <p>
 * Provides the structure for how segments are developed.
 * The boolean complete is a flag that notifies the Autonomous logic that
 * this segment is complete, and the next segment in the series can start.
 */
public abstract class AutonSegment {
	/**
	 * Flag to determine whether the current autonomous segment is complete.
	 */
	protected boolean complete;

	/**
	 * Default constructor (no-args)
	 */
	public AutonSegment() {
		this.complete = false;
	}

	/**
	 * Returns whether this segment is completed or not.
	 *
	 * @return complete
	 */
	public boolean isComplete() {
		return complete;
	}

	/**
	 * Sets the state of completion of the segment.
	 *
	 * @param complete
	 */
	public void setComplete(boolean complete) {
		this.complete = complete;
	}

	/**
	 * Imitates the WPILib interface Robot.autonomousInit().
	 * This method is called when this segment becomes the current segment of the routine.
	 */
	public abstract void autonomousInit();

	/**
	 * Imitates the WPILib interface Robot.autonomousPeriodic().
	 * This method is called multiple times per second while this segment is the current segment of the routine.
	 */
	public abstract void autonomousPeriodic();

	/**
	 * Optional method that can be overloaded in derived classes
	 */
	public void disabledInit(){
		//default disabledInit()
	};
}
