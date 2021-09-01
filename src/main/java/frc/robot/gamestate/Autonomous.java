package frc.robot.gamestate;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.gamestate.routine.*;
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

        this.autonRoutines.put("redBasic", new AutonRoutine(sparkDrive)
            .addSegment(new AutonShoot(teleoperated, manipulator, 3))
            .addSegment(new RotateToAngle(0, sparkDrive, teleopFunctions))
            //.addSegment(new AutonDrive(0.4, sparkDrive))
        );

//        this.autonRoutines.put("redTrenchRun", new AutonRoutine(sparkDrive)
//            .deriveFrom(autonRoutines.get("redBasic"))
//            .addSegment(new AutonTrajectory(sparkDrive, "TODO"))
//            .addSegment(new AutonIntake(manipulator, 3.0))
//            .addSegment(new RotateToAngle(0, sparkDrive, teleopFunctions))
//            .addSegment(new AutonShoot(teleoperated, manipulator, 5))
//        );
//
//        this.autonRoutines.put("redRendezVous", new AutonRoutine(sparkDrive)
//            .deriveFrom(autonRoutines.get("redBasic"))
//            .addSegment(new AutonTrajectory(sparkDrive, "TODO"))
//            .addSegment(new AutonIntake(manipulator, 3.0))
//            .addSegment(new RotateToAngle(0, sparkDrive, teleopFunctions))
//            .addSegment(new AutonShoot(teleoperated, manipulator, 5))
//        );

        var routineContainer = this.autonRoutines.keySet().stream().findFirst();
        if(routineContainer.isEmpty()) {
            throw new IllegalStateException("No autonomous routines defined!");
        }

        selectedRoutine = routineContainer.get();

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

        sparkDrive.getGyroscope().reset();

        manipulator.getShooter().setVisionLight(true);
        manipulator.getShooter().setHoodPercentOutput(0.25);
        manipulator.getShooter().setLowerLimitHit(false);
        manipulator.getShooter().setUpperLimitHit(false);
        manipulator.getShooter().setReadyToAim(false);

        selectedRoutine = autonChooser.getSelected();
        if(selectedRoutine == null) {
            System.out.println("No autonomous routine selected! Robot will not move...");
            return;
        }

        // Call init method for first autonomous segment in the routine
        if(!autonRoutines.containsKey(selectedRoutine)) return;
        autonRoutines.get(selectedRoutine).autonomousInit();
    }

    /**
     * Called directly from Robot.autonomousPeriodic() function. Runs the routine's segments
     * in order of how they were added.
     */
    public void autonomousPeriodic() {
        manipulator.getShooter().hoodCalibration();

        if(selectedRoutine == null) return;
        if(!autonRoutines.containsKey(selectedRoutine)) return;

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
        autonRoutines.values().forEach(AutonRoutine::disabledInit);
    }
}