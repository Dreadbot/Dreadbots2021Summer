package frc.robot.gamestate;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.gamestate.routine.AutonRoutine;
import frc.robot.gamestate.routine.AutonTrajectory;
import frc.robot.subsystem.Manipulator;
import frc.robot.subsystem.SparkDrive;
import frc.robot.utility.TeleopFunctions;

import java.util.HashMap;

/**
 * Logic Container for the Autonomous Period and Infinite Recharge at Home Challenges.
 */
public class Autonomous {
    // Routine Data
    private final HashMap<String, AutonRoutine> autonRoutines;

    private final SparkDrive sparkDrive;
    private final TeleopFunctions teleopFunctions;
    private final Manipulator manipulator;
    private final Teleoperated teleoperated;

    // SmartDashboard
    private final SendableChooser<String> autonChooser;
    private String selectedRoutine;

    /**
     * Default Constructor (no-args)
     */
    public Autonomous(SparkDrive sparkDrive, Manipulator manipulator, Teleoperated teleoperated) {
        this.teleoperated = teleoperated;
        this.manipulator = manipulator;
        this.sparkDrive = sparkDrive;
        this.teleopFunctions = teleoperated.getTeleopFunctions();

        this.autonRoutines = new HashMap<>();

        // Barrel Run Path
        this.autonRoutines.put("barrel", new AutonRoutine(sparkDrive)
            .addSegment(new AutonTrajectory(
                sparkDrive, "paths/path_feet_0.wpilib.json"))
        );

        // Bounce Path
        this.autonRoutines.put("bounce", new
            AutonRoutine(sparkDrive)
            .addSegment(new AutonTrajectory(
                sparkDrive, "paths/bounce_start.wpilib.json"))
            .addSegment(new AutonTrajectory(
                sparkDrive, "paths/bounce_first.wpilib.json"))
            .addSegment(new AutonTrajectory(
                sparkDrive, "paths/bounce_second.wpilib.json"))
            .addSegment(new AutonTrajectory(
                sparkDrive, "paths/bounce_final.wpilib.json"))
        );

        // Slalom Path
        this.autonRoutines.put("slalom", new AutonRoutine(sparkDrive)
            .addSegment(new AutonTrajectory(
                sparkDrive, "paths/slalom.wpilib.json"))
        );

        this.selectedRoutine = "bounce";

        this.autonChooser = new SendableChooser<>();
        this.autonChooser.setDefaultOption(selectedRoutine, selectedRoutine);
        for (String key : this.autonRoutines.keySet()) {
            this.autonChooser.addOption(key, key);
        }

        SmartDashboard.putData(autonChooser);
    }

    /**
     * Called directly from Robot.autonomousInit() function. Initializes the first
     * segment
     */
    public void autonomousInit() {
        System.out.println("Autonomous.autonomousInit");

        selectedRoutine = autonChooser.getSelected();

        sparkDrive.getGyroscope().reset();

        // Call init method for first autonomous segment in the routine
        autonRoutines.get(selectedRoutine).autonomousInit();

        manipulator.getShooter().setVisionLight(true);
        manipulator.getShooter().setHoodPercentOutput(0.25);
        manipulator.getShooter().setLowerLimitHit(false);
        manipulator.getShooter().setUpperLimitHit(false);
        manipulator.getShooter().setReadyToAim(false);
    }

    /**
     * Called directly from Robot.autonomousPeriodic() function. Runs the routine's segments
     * in order of how they were added.
     */
    public void autonomousPeriodic() {
        manipulator.getShooter().hoodCalibration();

        // Run the current segment's autonomousPeriodic() code.
        autonRoutines.get(selectedRoutine).autonomousPeriodic();
    }

    /**
     * Called directly from Robot.disabledInit() function. Resets the routine management
     * data so that the routine can be run in the same on/off cycle of the robot.
     */
    public void disabledInit() {
        // Performs a reset of all the routine data so it can be run multiple times
        // in the same on/off cycle of the robot.
        autonRoutines.get(selectedRoutine).disabledInit();
    }
}