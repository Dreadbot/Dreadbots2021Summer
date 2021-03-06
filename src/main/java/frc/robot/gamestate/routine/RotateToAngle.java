package frc.robot.gamestate.routine;

import frc.robot.subsystem.SparkDrive;
import frc.robot.utility.TeleopFunctions;

public class RotateToAngle extends AutonSegment{
    private int turnToAngle;

    private final SparkDrive sparkDrive;
    private TeleopFunctions teleopFunctions;

    public RotateToAngle(int turnToAngle, SparkDrive sparkDrive, TeleopFunctions teleopFunctions) {
        this.turnToAngle = -turnToAngle;
        this.sparkDrive = sparkDrive;
        this.teleopFunctions = teleopFunctions;
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
        teleopFunctions.WPITurnToAngle(turnToAngle);

        if(teleopFunctions.getTurnStatus()) {
            teleopFunctions.setTurnStatus(false);
            sparkDrive.stop();
            complete = true;
        }
    }
    
}
