package frc.robot.utility;

import edu.wpi.first.wpilibj.util.Color;

public class DreadbotConstants {
    // Motor Ids
    public static final int INTAKE_MOTOR_ID = 5;
    public static final int GENEVA_MOTOR_ID = 6;
    public static final int FLY_WHEEL_MOTOR_ID = 7;
    public static final int AIM_MOTOR_ID = 8;
    public static final int COLOR_WHEEL_MOTOR_ID = 9;

    // Solenoids
    public static final int INTAKE_PIN_ID = 0;
    public static final int PUNCH_SOLENOID_ID = 2;
    public static final int COLOR_WHEEL_SOLENOID_ID = 4;

    //Ultra
    public static final int ULTRA_PING_CHANNEL_ID = 6;
    public static final int ULTRA_ECHO_CHANNEL_ID = 7;

    // Feeder
    public static final int GENEVA_LIMIT_SWITCH_PORT = 9;
    public static final int PUNCH_LIMIT_SWITCH_PORT = 3;

    //DRIVE CONSTANTS
    public static final double DRIVE_SPEED_MULTIPLIER = 0.75;

    public static final double metersPerSecondToRevolutionsPerMinute = 877.19d;
    public static final double revolutionsPerMinuteToMetersPerSecond = 0.00114;

    public static final double metersToRevolutions = 14.64;
    public static final double revolutionsToMeters = 0.0683;

    //Color ids
    public static final Color kBlueTarget = new Color(0.143,0.427,0.429);
    public static final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
    public static final Color kRedTarget = new Color(0.561, 0.232, 0.114);
    public static final Color kYellowTarget = new Color(0.361, 0.524, 0.113);
}
