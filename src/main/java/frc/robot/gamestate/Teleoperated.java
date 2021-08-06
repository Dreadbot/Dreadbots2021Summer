package frc.robot.gamestate;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystem.Climber;
import frc.robot.subsystem.Manipulator;
import frc.robot.subsystem.SparkDrive;
import frc.robot.subsystem.SparkDrive.DriveMode;
import frc.robot.utility.DreadbotController;
import frc.robot.utility.TeleopFunctions;
import frc.robot.subsystem.ColorWheel;
import edu.wpi.first.wpilibj.util.Color;

public class Teleoperated {
    private final DreadbotController primaryJoystick;
    private final DreadbotController secondaryJoystick;
    private final Manipulator manipulator;
    private final Climber climber;
    private final SparkDrive sparkDrive;
    private final TeleopFunctions teleopFunctions;
    private final ColorWheel colorWheel;

    private AimShootStates aimShootState;

    final int maxAimCounts;
    int aimCounts;

    double selectedAngle = 0;
    int lastCount = 0;
    int staleCount = 0;

    double distance = 120;
    double rotSpeed = 0;

    boolean firstAim = true;

    Color targetColor;

    public Teleoperated(DreadbotController primaryJoystick,
                        DreadbotController secondaryJoystick,
                        Manipulator manipulator,
                        SparkDrive sparkDrive,
                        Climber climber, 
                        ColorWheel colorWheel) {
        this.primaryJoystick = primaryJoystick;
        this.secondaryJoystick = secondaryJoystick;

        this.manipulator = manipulator;
        this.sparkDrive = sparkDrive;
        this.climber = climber;
        this.teleopFunctions = new TeleopFunctions(secondaryJoystick, manipulator, sparkDrive);
        this.colorWheel = colorWheel;

        aimShootState = AimShootStates.AIMING;
        maxAimCounts = 150;
        aimCounts = 0;
    }

    public void initIntake() {
        manipulator.getIntake().deployIntake();
    }

    public void initDrive() {
        sparkDrive.getGyroscope().zeroYaw();
    }

    public void teleopIntake() {
        if (secondaryJoystick.isXButtonPressed()) {
            manipulator.getIntake().setSpeed(-4000);
        } else if (secondaryJoystick.isAButtonPressed()) {
            manipulator.getIntake().setSpeed(4000);
        } else {
            manipulator.getIntake().setPercentOutput(0);
        }
    }

    public void teleopDrive() {
        // Get the DriveMode from controller input.
        DriveMode driveMode = DriveMode.fromDriverInput(primaryJoystick);

        sparkDrive.drive(primaryJoystick.getYAxis(),
            primaryJoystick.getZAxis(),
            driveMode);
    }

    public void teleopShooter() {
        SmartDashboard.putNumber("Current Angle", sparkDrive.getGyroscope().getYaw());
        double hood_position = SmartDashboard.getNumber("Hood Position", 0.5);

        if (SmartDashboard.getNumber("detectionCount", lastCount) == lastCount)
            staleCount++;
        else
            staleCount = 0;

        if (aimShootState == AimShootStates.AIMING) {
            distance = SmartDashboard.getNumber("selectedDistance", 120);
        }
        //update the latest count, for use on next loop iteration
        lastCount = (int) SmartDashboard.getNumber("detectionCount", lastCount);

        //if we are done turning (not currently turning), then update angle from vision
        if (teleopFunctions.getTurnStatus() && firstAim) {
            selectedAngle = SmartDashboard.getNumber("selectedAngle", 0.0);
            firstAim = false;
        }
        //Only turn and shoot when we hold the button, and we have seen the target recently
        if (secondaryJoystick.isYButtonPressed()) {
        	double shooting_hood_position = SmartDashboard.getNumber("Hood Position", 0.5);
            double shooting_rpm = SmartDashboard.getNumber("Shooter Target Speed", 4000);
            System.out.println("Cont Shooting");
            manipulator.continuousShoot(shooting_hood_position, 0.4, shooting_rpm);
            SmartDashboard.putNumber("camNumber", 0);
        } else if (secondaryJoystick.isBButtonPressed() && staleCount < 5) {
            aimingContinuousShoot(distance, selectedAngle, 0.4);
            SmartDashboard.putNumber("camNumber", 0);
            staleCount = 0;
        } else if (secondaryJoystick.isBButtonPressed()) {
            SmartDashboard.putNumber("camNumber", 0);
        } else if (secondaryJoystick.isRightBumperPressed()) {
            manipulator.sensorAdvanceGeneva(true, true);
        } else if (secondaryJoystick.isLeftBumperPressed()) {
            manipulator.sensorAdvanceGeneva(true, false);
        } else if (manipulator.getSensorAdvanceGenevaState() == 2) { //2 means Geneva is stopped
            manipulator.resetManipulatorElements();
            teleopFunctions.setTurnStatus(true);
            aimCounts = 0;
            aimShootState = AimShootStates.AIMING;
            rotSpeed = 0;
            sparkDrive.getGyroscope().reset();
            firstAim = true;
        } else {
            manipulator.resetManipulatorElements();
        }
    }

    public void teleopClimber() {
        if(secondaryJoystick.isStartButtonPressed()) {
            climber.SetTelescope(true);
        } else if(secondaryJoystick.isBackButtonPressed()) {
            climber.SetTelescope(false);
        }

        if(secondaryJoystick.isRightTriggerPressed()) {
            climber.SetWinch(-0.5);
        } else if(secondaryJoystick.isLeftTriggerPressed()) {
            climber.SetWinch(0.5);
        } else {
            climber.SetWinch(0.0);
        }
    }

    public int aimingContinuousShoot(double distance, double targetAngle, double genevaSpeed) {
        int numPunches = 0;
		double rpm = manipulator.getSelectedRPM(distance);
        SmartDashboard.putNumber("Target Shooting Velocity", rpm);
		double hoodPosition = manipulator.getSelectedHoodPosition(distance);

        aimShootState = (aimCounts < maxAimCounts) ? AimShootStates.AIMING : AimShootStates.SHOOTING;
        SmartDashboard.putNumber("aimShootState", aimShootState.ordinal());
        switch (aimShootState) {
            case AIMING:
                // rotSpeed = teleopFunctions.calculateTurnToAngle(targetAngle);
                teleopFunctions.WPITurnToAngle(targetAngle);
                manipulator.prepareShot(-rpm, hoodPosition);
                break;
            case SHOOTING:
                sparkDrive.stop();
                numPunches = manipulator.continuousShoot(hoodPosition, genevaSpeed, rpm);
                break;
        }
        aimCounts++;
        return numPunches;
    }

    public void resetAimCounts() {
        aimCounts = 0;
    }

    public TeleopFunctions getTeleopFunctions() {
        return teleopFunctions;
    }

    public enum AimShootStates {
        AIMING,
        SHOOTING;
    }

    public void setAimShootState(AimShootStates a) {
        aimShootState = a;
    }

    public AimShootStates getAimShootStates() {
        return aimShootState;
    }
    public void colorWheelControls(){
        targetColor = colorWheel.ColorDataGetter();
        if(primaryJoystick.isRightBumperPressed()){
            colorWheel.colorWheelExtension.set(true);
        }
        else if (primaryJoystick.isRightTriggerPressed()){
            colorWheel.colorWheelExtension.set(false);
        }

        if(primaryJoystick.isLeftBumperPressed() && targetColor != null){
            colorWheel.RotateToColor(targetColor);
        }
        
    }

}
