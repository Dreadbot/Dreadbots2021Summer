package frc.robot.gamestate.routine;

import edu.wpi.first.wpilibj.Timer;

/**
 * Derived class of AutonSegment that acts as a delay during an autonomous period.
 */
public class AutonTimer extends AutonSegment {
	// Timer util class used to determine time passed.
	private Timer timer;

	// The delay amount in seconds
	private double timerDelaySeconds;

	/**
	 * Constructor for AutonTimer
	 *
	 * @param timerDelaySeconds Delay amount in seconds
	 */
	public AutonTimer(double timerDelaySeconds) {
		this.timerDelaySeconds = timerDelaySeconds;

		timer = new Timer();
	}

	@Override
	public void autonomousInit() {
		System.out.println("AutonTimer.autonomousInit");

		// Start the timer (counts up)
		timer.start();
	}

	@Override
	public void autonomousPeriodic() {
		// If the timer has met or exceeded the amount of delay specified,
		// Reset the timer, stop the timer, and indicate to the Autonomous logic
		// that this segment is complete.
		if(timer.get() >= timerDelaySeconds) {
			timer.reset();
			timer.stop();
			complete = true;
		}
	}
}
