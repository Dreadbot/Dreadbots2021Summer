package frc.robot.gamestate.routine;

import java.util.ArrayList;

import frc.robot.subsystem.SparkDrive;

public class AutonRoutine {
    private ArrayList<AutonSegment> segments;
    private int segmentIndex;
    private boolean completed;
    private SparkDrive sparkDrive;

    public AutonRoutine(SparkDrive sparkDrive) {
        segments = new ArrayList<>();
        segmentIndex = 0;
        completed = false;

        this.sparkDrive = sparkDrive;
    }
    
    public AutonRoutine addSegment(AutonSegment segment) {
        segments.add(segment);

        return this;
    }
    
    public void autonomousInit() {
        sparkDrive.getGyroscope().reset();

        // Call init method for first autonomous segment in the routine
        segments.get(segmentIndex).autonomousInit();
    }

    public void autonomousPeriodic(){
        // Prevent IndexOutOfBoundsExceptions and allows the robot to remain
		// running after the routine is finished.
        if(completed)
            return;
        
        // Run the current segment's autonomousPeriodic() code.
        segments.get(segmentIndex).autonomousPeriodic();

        // Check to see if the current segment's task has been completed
        if(segments.get(segmentIndex).isComplete()) {
            // Move to the next segment of the routine
            segmentIndex++;

            // If there are no more segments in the routine, stop the execution
			// of the autonomous logic.
            if(segmentIndex >= segments.size()) {
                // Prevents the autonomous logic from being run until the next time
				// the autonomous period starts.
                completed = true;
                return;
            }
            
            // If there are more segments in the routine,
			// call the next segment's init method.
            segments.get(segmentIndex).autonomousInit();
        }
    }

    /**
	 * Called directly from Robot.disabledInit() function. Resets the routine management
	 * data so that the routine can be run in the same on/off cycle of the robot.
	 */
	public void disabledInit() {
		// Performs a reset of all the routine data so it can be run multiple times
		// in the same on/off cycle of the robot.
		segmentIndex = 0;
		completed = false;
		for (AutonSegment segment : segments) {
			segment.setComplete(false);
			segment.disabledInit();
		}
	}

    public boolean isCompleted() {
        return completed;
    }

    public void setCompleted(boolean completed) {
        this.completed = completed;
    }
}