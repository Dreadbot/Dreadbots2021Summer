// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.gamestate.Autonomous;
import frc.robot.gamestate.Teleoperated;
import frc.robot.subsystem.Climber;
import frc.robot.subsystem.Feeder;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Manipulator;
import frc.robot.subsystem.Shooter;
import frc.robot.subsystem.SparkDrive;
import frc.robot.utility.DreadbotController;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    // SUBSYSTEMS
    public SparkDrive sparkDrive = new SparkDrive();
    public Shooter shooter = new Shooter();
    public Intake intake = new Intake();
    public Feeder feeder = new Feeder();
    public Manipulator manipulator = new Manipulator(intake, feeder, shooter);

    public Climber climber = new Climber();

    // JOYSTICKS
    public DreadbotController primaryJoystick = new DreadbotController(0);
    public DreadbotController secondaryJoystick = new DreadbotController(1);

    // GAME STATE
    private final Teleoperated teleoperated = new Teleoperated(primaryJoystick, secondaryJoystick, manipulator, sparkDrive, climber);
    private Autonomous autonomous = new Autonomous(sparkDrive, manipulator, teleoperated);

    @Override
    public void robotInit() { }

    @Override
    public void robotPeriodic() {
        sparkDrive.periodic();
    }

    @Override
    public void autonomousInit() {
        autonomous.autonomousInit();
    }

    @Override
    public void autonomousPeriodic() {
        autonomous.autonomousPeriodic();
    }

    @Override
    public void teleopInit() {
//         SmartDashboard Setup
         SmartDashboard.putNumber("Shooter P", .0025);
         SmartDashboard.putNumber("Shooter I", 3.3e-7);
         SmartDashboard.putNumber("Shooter D", 0.03);
         SmartDashboard.putNumber("Shooter Target Speed", 3550);

         // Setup shooter for teleop
         shooter.setVisionLight(true);
         shooter.setHoodPercentOutput(0.25);
         shooter.setUpperLimitHit(false);
         shooter.setLowerLimitHit(false);
         shooter.setReadyToAim(false);

         sparkDrive.getGyroscope().reset();

         intake.deployIntake();
    }

    @Override
    public void teleopPeriodic() {
        // // Drive
         teleoperated.teleopDrive();

        // // Shooter
         shooter.setPID(SmartDashboard.getNumber("Shooter P", .0025),
             SmartDashboard.getNumber("Shooter I", 3.3e-7),
             SmartDashboard.getNumber("Shooter D", 0.03));
         SmartDashboard.putNumber("Shooter RPM", manipulator.getShooter().getShootingSpeed());

         shooter.hoodCalibration();

         SmartDashboard.putNumber("Shooter Velocity (Actual)", shooter.getShootingSpeed());
         teleoperated.teleopShooter();

        // // Intake
        teleoperated.teleopIntake();

        teleoperated.teleopClimber();
    }

    @Override
    public void disabledInit() {
        autonomous.disabledInit();

        climber.SetWinch(0.0);

        shooter.setVisionLight(false);
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }
}
