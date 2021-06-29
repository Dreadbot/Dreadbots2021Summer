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
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.utility.DreadbotMath;

public class Drive {
    private static final MotorType K_MOTORTYPE = CANSparkMaxLowLevel.MotorType.kBrushless;

    // Drive Constants
    private static final double K_MAX_SPEED = 3.0d; // meters per second
    private static final double K_MAX_ACCELERATION = 1.5d; // meters per second squared
    private static final double K_MAX_ANGULAR_SPEED = 2.0d * Math.PI; // radians per second

    // Voltage Feedforward Constants
    private static final double K_S = 0.229d; // volts
    private static final double K_V = 0.0437d; // volt seconds per meter
    private static final double K_A = 0.00512d; // volt seconds squared per meter
    private static final double K_P = 1.19e-10d; // meters per second
    
    // Robot Details
    private static final double kTrackwidthMeters = 0.705d; // meters

    // Motors
    private final CANSparkMax leftFrontMotor = new CANSparkMax(0, K_MOTORTYPE);
    private final CANSparkMax leftBackMotor = new CANSparkMax(1, K_MOTORTYPE);
    private final CANSparkMax rightFrontMotor = new CANSparkMax(2, K_MOTORTYPE);
    private final CANSparkMax rightBackMotor = new CANSparkMax(3, K_MOTORTYPE);

    // Odometry
    private final DifferentialDriveOdometry odometry;

    // Gyroscope
    private final AHRS gyroscope = new AHRS(SerialPort.Port.kUSB);

    // Slew Rate Limiters (Prevent unintentional erratic movements) at (1/3)sec
    private final SlewRateLimiter speedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

    // Drive calculations class
    private final DifferentialDrive robotDrive = new DifferentialDrive(this.leftFrontMotor, this.rightFrontMotor);

    public Drive() {
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

        // Set up gyroscope
        this.gyroscope.reset();

        // Set up the PID Controllers
        Drive.pidControllerSetup(leftFrontMotor);
        Drive.pidControllerSetup(leftBackMotor);
        Drive.pidControllerSetup(rightFrontMotor);
        Drive.pidControllerSetup(rightBackMotor);

        // Set up odometry
        this.odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyroscope.getYaw()));
    }

    public void periodic() {
        final double leftPosition = getLeftFrontMotorEncoder().getPosition() * 0.0683;
        final double rightPosition = getRightFrontMotorEncoder().getPosition() * 0.0683;
        final var newPose = odometry.update(gyroscope.getRotation2d(),
                leftPosition, -rightPosition);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            getLeftFrontMotorEncoder().getVelocity() * 0.00114,
            -getRightFrontMotorEncoder().getVelocity() * 0.00114
        );
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, gyroscope.getRotation2d());
    }

    public void driveVolts(double leftVolts, double rightVolts) {
        leftFrontMotor.setVoltage(leftVolts);
        leftBackMotor.setVoltage(leftVolts);
        rightFrontMotor.setVoltage(rightVolts);
        rightBackMotor.setVoltage(rightVolts);
    }

    public void resetEncoders() {
        getLeftFrontMotorEncoder().setPosition(0.0);
        getLeftBackMotorEncoder().setPosition(0.0);
        getRightFrontMotorEncoder().setPosition(0.0);
        getRightBackMotorEncoder().setPosition(0.0);
    }

    public double getAverageEncoderDistance() {
        return (getLeftFrontMotorEncoder().getPosition() + getRightFrontMotorEncoder().getPosition()) * (0.5 * 0.0683);
    }

    public void zeroHeading() {
        gyroscope.reset();
    }

    public double getHeading() {
        return gyroscope.getYaw();
    }

    public double getTurnRate() {
        return -gyroscope.getRate();
    }

    public void stop() {
        leftFrontMotor.set(0.0);
        leftBackMotor.set(0.0);
        rightFrontMotor.set(0.0);
        rightBackMotor.set(0.0);
    }

    /**
     * New drive method as recommended by WPILib, which is velocity-based over percentage output.
     *
     * @param xSpeed forward/backward speed input
     * @param zRotation rotational input
     */
    public void drive(double xSpeed, double zRotation) {
        this.drive(xSpeed, zRotation, DriveMode.NORMAL);
    }

    /**
     * New drive method as recommended by WPILib, which is velocity-based over percentage output.
     *
     * @param xSpeed forward/backward speed input
     * @param zRotation rotational input
     * @param driveMode The drive mode setting (final multiplier).
     */
    public void drive(double xSpeed, double zRotation, final DriveMode driveMode) {
        this.drive(xSpeed, zRotation, driveMode, 0.09);
    }

    /**
     * New drive method as recommended by WPILib, which is velocity-based over percentage output.
     *
     * @param xSpeed forward/backward speed input
     * @param zRotation rotational input
     * @param driveMode The drive mode setting (final multiplier).
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
