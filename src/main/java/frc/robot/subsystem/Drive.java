package frc.robot.subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drive {
    private CANSparkMax leftFrontMotor;
    private CANSparkMax leftBackMotor;
    private CANSparkMax rightFrontMotor;
    private CANSparkMax rightBackMotor;

    private final DifferentialDrive robotDrive;

    public Drive() {
        this.leftFrontMotor = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.leftBackMotor = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.rightFrontMotor = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.rightBackMotor = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);

        // Back motors follow front motors 
        this.leftBackMotor.follow(this.leftFrontMotor);
        this.rightBackMotor.follow(this.rightFrontMotor);

        this.robotDrive = new DifferentialDrive(this.leftFrontMotor, this.rightFrontMotor);
    }

    public void drive(double xSpeed, double zRotation) {
        this.robotDrive.arcadeDrive(xSpeed, zRotation);
    }
}
