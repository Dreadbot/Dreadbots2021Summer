package frc.robot.subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drive {
    private static final MotorType K_MOTORTYPE = CANSparkMaxLowLevel.MotorType.kBrushless;

    // Drive Constants
    private static final double K_MAX_SPEED = 3.0d; // meters per second
    private static final double K_MAX_ANGULAR_SPEED = 2.0d * Math.PI; // radians per second

    public static final double kTrackwidthMeters = 0.705d; // meters

    // Motors
    private CANSparkMax leftFrontMotor = new CANSparkMax(0, K_MOTORTYPE);
    private CANSparkMax leftBackMotor = new CANSparkMax(1, K_MOTORTYPE);
    private CANSparkMax rightFrontMotor = new CANSparkMax(2, K_MOTORTYPE);
    private CANSparkMax rightBackMotor = new CANSparkMax(3, K_MOTORTYPE);

    // Slew Rate Limiters (Prevent unintentional erratic movements) at (1/3)sec
    private final SlewRateLimiter speedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

    // Drive calculations class
    private final DifferentialDrive robotDrive = new DifferentialDrive(this.leftFrontMotor, this.rightFrontMotor);

    public Drive() {
        // Back motors follow front motors 
        this.leftBackMotor.follow(this.leftFrontMotor);
        this.rightBackMotor.follow(this.rightFrontMotor);
    }

    public void drive(double xSpeed, double zRotation) {
        // Apply a slew rate limiter and use value as a percentage of max speed.
        xSpeed = -speedLimiter.calculate(xSpeed) * K_MAX_SPEED;

        // Apply a slew rate limiter and calculate the angular velocity
        zRotation = -rotLimiter.calculate(zRotation) * K_MAX_ANGULAR_SPEED;

        this.robotDrive.arcadeDrive(xSpeed, zRotation);
    }
}
