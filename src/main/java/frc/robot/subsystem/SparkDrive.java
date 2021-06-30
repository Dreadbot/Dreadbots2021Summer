package frc.robot.subsystem;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.utility.DreadbotConstants;
import frc.robot.utility.DreadbotMath;

public class SparkDrive {
    private static final MotorType K_MOTORTYPE = CANSparkMaxLowLevel.MotorType.kBrushless;

    // Drive Constants
    public static final double K_MAX_SPEED = 3.0d; // meters per second
    public static final double K_MAX_ACCELERATION = 1.5d; // meters per second squared
    public static final double K_MAX_ANGULAR_SPEED = 2.0d * Math.PI; // radians per second

    // Voltage Feedforward Constants
    public static final double K_S = 0.229d; // volts
    public static final double K_V = 0.0437d; // volt seconds per meter
    public static final double K_A = 0.00512d; // volt seconds squared per meter
    public static final double K_P = 1.19e-10d; // meters per second

    // Robot Details
    public static final double kTrackwidthMeters = 0.705d; // meters

    // Motors
    private final CANSparkMax leftFrontMotor = new CANSparkMax(0, K_MOTORTYPE);
    private final CANSparkMax leftBackMotor = new CANSparkMax(1, K_MOTORTYPE);
    private final CANSparkMax rightFrontMotor = new CANSparkMax(2, K_MOTORTYPE);
    private final CANSparkMax rightBackMotor = new CANSparkMax(3, K_MOTORTYPE);

    // Gyroscope
    private final AHRS gyroscope = new AHRS(SerialPort.Port.kUSB);

    // Slew Rate Limiters (Prevent unintentional erratic movements) at (1/3)sec
    private final SlewRateLimiter speedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

    // Drive calculations class
    private final DifferentialDrive robotDrive = new DifferentialDrive(this.leftFrontMotor, this.rightFrontMotor);
    private final DifferentialDriveOdometry odometry;
    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    public SparkDrive() {
        // Back motors follow front motors 
        this.leftBackMotor.follow(this.leftFrontMotor);
        this.rightBackMotor.follow(this.rightFrontMotor);

        // Set up motor controllers
        this.leftFrontMotor.restoreFactoryDefaults();
        this.leftBackMotor.restoreFactoryDefaults();
        this.rightFrontMotor.restoreFactoryDefaults();
        this.rightBackMotor.restoreFactoryDefaults();
        this.leftFrontMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        this.leftBackMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        this.rightFrontMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        this.rightBackMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        // Set up the PID Controllers
        SparkDrive.pidControllerSetup(leftFrontMotor);
        SparkDrive.pidControllerSetup(leftBackMotor);
        SparkDrive.pidControllerSetup(rightFrontMotor);
        SparkDrive.pidControllerSetup(rightBackMotor);

        // Set up gyroscope
        this.gyroscope.reset();

        // Set up odometry
        this.odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyroscope.getYaw()));
    }

    /**
     * Updates the robot drivetrain odometry, using the current encoder positions to calculate
     * where the robot is.
     */
    public void periodic() {
        final double leftPosition = getLeftFrontMotorEncoder().getPosition() * DreadbotConstants.revolutionsToMeters;
        final double rightPosition = getRightFrontMotorEncoder().getPosition() * DreadbotConstants.revolutionsToMeters;
        odometry.update(gyroscope.getRotation2d(), leftPosition, -rightPosition);
    }

    /**
     * Get the current position of the robot in meters 2D (x, y) through the robot drivetrain
     * odometry.
     *
     * @return Pose2d object with x & y components of the robot's position.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Returns a DifferentialDriveWheelSpeeds object with the values of the speeds of the wheels.
     *
     * @return DifferentialDriveWheelSpeeds object with wheel speeds of the drivetrain.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            getLeftFrontMotorEncoder().getVelocity() * DreadbotConstants.revolutionsPerMinuteToMetersPerSecond,
            -getRightFrontMotorEncoder().getVelocity() * DreadbotConstants.revolutionsPerMinuteToMetersPerSecond
        );
    }

    /**
     * Resets the odometry calculator, usually used after a trajectory following task has been
     * completed.
     *
     * @param pose The position of the robot on the field
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, gyroscope.getRotation2d());
    }

    /**
     * Drive method that takes a desired motor controller voltage feedforward as input.
     *
     * @param leftVolts  Amount of voltage to be applied to each left side motor controller.
     * @param rightVolts Amount of voltage to be applied to each right side motor controller.
     */
    public void driveVolts(double leftVolts, double rightVolts) {
        leftFrontMotor.setVoltage(leftVolts);
        leftBackMotor.setVoltage(leftVolts);
        rightFrontMotor.setVoltage(rightVolts);
        rightBackMotor.setVoltage(rightVolts);
    }

    /**
     * Resets the motor controller encoders back to a zero position.
     */
    public void resetEncoders() {
        getLeftFrontMotorEncoder().setPosition(0.0);
        getLeftBackMotorEncoder().setPosition(0.0);
        getRightFrontMotorEncoder().setPosition(0.0);
        getRightBackMotorEncoder().setPosition(0.0);
    }

    /**
     * Gets the Average distance recorded by the motor controller encoders for a general idea of how
     * far the robot has moved
     *
     * @return Average distance traveled recorded between the left and right sides of the drivetrain.
     */
    public double getAverageEncoderDistance() {
        return (getLeftFrontMotorEncoder().getPosition() + getRightFrontMotorEncoder().getPosition()) * (0.5 * DreadbotConstants.revolutionsToMeters);
    }

    /**
     * Resets the gyroscope to 0 degrees.
     */
    public void resetGyroscope() {
        gyroscope.reset();
    }

    /**
     * Gets the current heading of the gyroscope. (Greater values are counter-clockwise)
     *
     * @return Current heading of the gyroscope.
     */
    public double getHeading() {
        return gyroscope.getYaw();
    }

    /**
     * Gets the current rate of change of the
     * heading measure of the gyroscope (Greater values are counter-clockwise).
     *
     * @return Current rate of change of the heading measure.
     */
    public double getTurnRate() {
        return -gyroscope.getRate();
    }

    /**
     * Stops all motors.
     */
    public void stop() {
        leftFrontMotor.set(0.0);
        leftBackMotor.set(0.0);
        rightFrontMotor.set(0.0);
        rightBackMotor.set(0.0);
    }

    /**
     * New drive method as recommended by WPILib, which is velocity-based over percentage output.
     *
     * @param xSpeed    The forward/backward factor of the drivetrain control.
     * @param zRotation The rotational factor of the drivetrain control.
     */
    public void drive(double xSpeed, double zRotation) {
        this.drive(xSpeed, zRotation, DriveMode.NORMAL);
    }

    /**
     * New drive method as recommended by WPILib, which is velocity-based over percentage output.
     *
     * @param xSpeed    The forward/backward factor of the drivetrain control.
     * @param zRotation The rotational factor of the drivetrain control.
     * @param driveMode The drive mode setting (final multiplier).
     */
    public void drive(double xSpeed, double zRotation, final DriveMode driveMode) {
        this.drive(xSpeed, zRotation, driveMode, 0.09);
    }

    /**
     * New drive method as recommended by WPILib, which is velocity-based over percentage output.
     *
     * @param xSpeed           The forward/backward factor of the drivetrain control.
     * @param zRotation        The rotational factor of the drivetrain control.
     * @param driveMode        The drive mode setting (final multiplier).
     * @param joystickDeadband The applied joystick deadband.
     */
    public void drive(double xSpeed, double zRotation, final DriveMode driveMode, final double joystickDeadband) {
        // Apply deadband
        xSpeed = DreadbotMath.applyDeadbandToValue(xSpeed, -joystickDeadband, joystickDeadband, 0.0d);
        zRotation = DreadbotMath.applyDeadbandToValue(zRotation, -joystickDeadband, joystickDeadband, 0.0d);

        // Apply a slew rate limiter and use value as a percentage of max speed.
        xSpeed = -speedLimiter.calculate(xSpeed) * K_MAX_SPEED * driveMode.finalValueMultiplier;

        // Apply a slew rate limiter and calculate the angular velocity
        zRotation = -rotLimiter.calculate(zRotation) * K_MAX_ANGULAR_SPEED * driveMode.finalValueMultiplier;

        this.robotDrive.arcadeDrive(xSpeed, zRotation);
    }

    /**
     * An improved and more readable version of the Dreadbot's homemade tank
     * drive function with default values for final value multiplier and joystick
     * deadband.
     * This function has since been deprecated, as we want to move toward the WPI
     * recommended drive code.
     *
     * @param forwardAxisFactor  The forward factor of the drivetrain control.
     * @param rotationAxisFactor The rotational factor of the drivetrain control.
     */
    @Deprecated
    public void tankDrive(double forwardAxisFactor,
                          double rotationAxisFactor) {
        tankDrive(forwardAxisFactor, rotationAxisFactor, DriveMode.NORMAL);
    }

    /**
     * An improved and more readable version of the Dreadbot's homemade tank
     * drive function with default values for the joystick deadband.
     * This function has since been deprecated, as we want to move toward the WPI
     * recommended drive code.
     *
     * @param forwardAxisFactor  The forward factor of the drivetrain control.
     * @param rotationAxisFactor The rotational factor of the drivetrain control.
     * @param driveMode          The drive mode setting (final multiplier).
     */
    @Deprecated
    public void tankDrive(double forwardAxisFactor,
                          double rotationAxisFactor,
                          final DriveMode driveMode) {
        tankDrive(forwardAxisFactor, rotationAxisFactor, driveMode, 0.09);
    }

    /**
     * An improved and more readable version of the Dreadbot's homemade tank
     * drive function.
     * This function has since been deprecated, as we want to move toward the WPI
     * recommended drive code.
     *
     * @param forwardAxisFactor  The forward factor of the drivetrain control.
     * @param rotationAxisFactor The rotational factor of the drivetrain control.
     * @param driveMode          The drive mode setting (final multiplier).
     * @param joystickDeadband   The applied joystick deadband.
     */
    @Deprecated
    public void tankDrive(double forwardAxisFactor,
                          double rotationAxisFactor,
                          final DriveMode driveMode,
                          final double joystickDeadband) {
        double[] speedControllerOutputs = new double[4];

        // Clamp Values to Acceptable Ranges (between -1.0 and 1.0).
        forwardAxisFactor = DreadbotMath.clampValue(forwardAxisFactor, -1.0d, 1.0d);
        rotationAxisFactor = DreadbotMath.clampValue(rotationAxisFactor, -1.0d, 1.0d);

        // Apply an Optional Joystick Deadband
        forwardAxisFactor = DreadbotMath.applyDeadbandToValue(forwardAxisFactor, -joystickDeadband, joystickDeadband, 0.0d);
        rotationAxisFactor = DreadbotMath.applyDeadbandToValue(rotationAxisFactor, -joystickDeadband, joystickDeadband, 0.0d);

        // Essential Drive Math based on the two movement factors.
        double leftFinalSpeed = -forwardAxisFactor + rotationAxisFactor;
        double rightFinalSpeed = forwardAxisFactor + rotationAxisFactor;

        // Assign each motor value to the output side it's on.
        speedControllerOutputs[0] = leftFinalSpeed;
        speedControllerOutputs[1] = rightFinalSpeed;
        speedControllerOutputs[2] = leftFinalSpeed;
        speedControllerOutputs[3] = rightFinalSpeed;

        // Add the final multiplier to the values.
        for (int i = 0; i < speedControllerOutputs.length; i++)
            speedControllerOutputs[i] *= driveMode.finalValueMultiplier;

        // Normalize the values to become between 1.0 and -1.0.
        DreadbotMath.normalizeValues(speedControllerOutputs);

        leftFrontMotor.set(speedControllerOutputs[0]);
        rightFrontMotor.set(speedControllerOutputs[1]);
        leftBackMotor.set(speedControllerOutputs[2]);
        rightBackMotor.set(speedControllerOutputs[3]);
    }

    private static void pidControllerSetup(CANSparkMax motor) {
        // All four motor controller PIDs should have the same setting.
        CANPIDController pidController = motor.getPIDController();
        pidController.setP(0.2);
        pidController.setI(1e-4);
        pidController.setD(1);
        pidController.setIZone(0.1);
        pidController.setFF(0);
        pidController.setOutputRange(-1, 1);
    }

    public CANSparkMax getLeftFrontMotor() {
        return leftFrontMotor;
    }

    public CANSparkMax getLeftBackMotor() {
        return leftBackMotor;
    }

    public CANSparkMax getRightFrontMotor() {
        return rightFrontMotor;
    }

    public CANSparkMax getRightBackMotor() {
        return rightBackMotor;
    }

    public CANEncoder getLeftFrontMotorEncoder() {
        return leftFrontMotor.getEncoder();
    }

    public CANEncoder getLeftBackMotorEncoder() {
        return leftBackMotor.getEncoder();
    }

    public CANEncoder getRightFrontMotorEncoder() {
        return rightFrontMotor.getEncoder();
    }

    public CANEncoder getRightBackMotorEncoder() {
        return rightBackMotor.getEncoder();
    }

    public AHRS getGyroscope() {
        return gyroscope;
    }

    /**
     * DriveMode is the enumeration of the default final value multipliers for teleop.
     */
    public enum DriveMode {
        TURBO(0.9),
        NORMAL(0.5),
        TURTLE(0.2),
        ADJUSTMENT(0.1);

        public double finalValueMultiplier;

        DriveMode(double finalValueMultiplier) {
            this.finalValueMultiplier = finalValueMultiplier;
        }
    }
}
