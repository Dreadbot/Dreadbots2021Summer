package frc.robot.subsystem;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

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
    }

    public void drive(double xSpeed, double zRotation) {
        // Apply a slew rate limiter and use value as a percentage of max speed.
        xSpeed = -speedLimiter.calculate(xSpeed) * K_MAX_SPEED;

        // Apply a slew rate limiter and calculate the angular velocity
        zRotation = -rotLimiter.calculate(zRotation) * K_MAX_ANGULAR_SPEED;

        this.robotDrive.arcadeDrive(xSpeed, zRotation);
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
}
