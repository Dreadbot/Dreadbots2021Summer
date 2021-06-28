// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.gamestate.Autonomous;
//import frc.robot.gamestate.Teleoperated;
import frc.robot.subsystem.*;
//import frc.robot.utility.Constants;
import frc.robot.utility.DreadbotController;
//import frc.robot.utility.logger.RobotLogger;

import java.util.ArrayList;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // SUBSYSTEMS
	public Drive drive;
	public Shooter shooter;
	//public Intake intake;
	//public Feeder feeder;
	//public Manipulator manipulator;
	//public Ultra sonic2;

	// JOYSTICKS
	public DreadbotController primaryJoystick;
	public DreadbotController secondaryJoystick;

	// TESTING ONLY
	//public ArrayList<Subsystem> testingSubsystems;
	public int currentTestingIndex;
	public boolean isTestingCompleted;

	// GAME STATE
	//private Autonomous autonomous;
  //private Teleoperated teleoperated;
  
  @Override
  public void robotInit() {

		// Joystick Initialization
		primaryJoystick = new DreadbotController(0);
		secondaryJoystick = new DreadbotController(1);

		// Subsystem Initialization
		drive = new Drive();
		//drive.tankDrive(0.1, 0.0);
		shooter = new Shooter();
		/*intake = new Intake();
		feeder = new Feeder();
		manipulator = new Manipulator(intake,
			feeder,
			shooter);*/

		//sonic1 = new Ultra(Constants.ULTRA_PING_CHANNEL_ID, Constants.ULTRA_ECHO_CHANNEL_ID);
		// sonic2 = new Ultra(6, 7);

		// Game State Initialization
		/*teleoperated = new Teleoperated(primaryJoystick,
			secondaryJoystick,
			manipulator,
			sparkDrive);
		autonomous = new Autonomous(sparkDrive, teleoperated.getTeleopFunctions(), manipulator, teleoperated);*/

		// Testing Initialization
		//testingSubsystems = new ArrayList<>();
		//testingSubsystems.add(sparkDrive);
		//testingSubsystems.add(manipulator);

  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
