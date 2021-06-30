package frc.robot.subsystem;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpiutil.math.MathUtil;

public class Shooter {
    // Limit Switch Variables
    int minHoodPosition;
    int maxHoodPosition;
    boolean readyToAim = false;
    boolean lowerLimitHit = false;
    boolean upperLimitHit = false;

    // Motor Controllers
    private final CANSparkMax shooterMotor = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax aimingMotor = new CANSparkMax(8, CANSparkMaxLowLevel.MotorType.kBrushless);

    // PID Controllers
    private final CANPIDController shooterMotorPID;
    private final CANPIDController aimingMotorPID;

    // Encoders
    private final CANEncoder shooterMotorEncoder;
    private final CANEncoder aimingMotorEncoder;

    // Limit Switches
    private final DigitalInput upperLimitSwitch = new DigitalInput(1);
    private final DigitalInput lowerLimitSwitch = new DigitalInput(2);

    // Vision Light
    private final Solenoid visionLEDRing = new Solenoid(7);

    // Shooting Mechanism Variables
    private double aimPosition;
    private double range;

    public Shooter() {
        shooterMotor.restoreFactoryDefaults();

        // Get the PID Controller Objects
        shooterMotorPID = shooterMotor.getPIDController();
        aimingMotorPID = aimingMotor.getPIDController();

        // Get the Encoder Objects
        shooterMotorEncoder = shooterMotor.getEncoder();
        aimingMotorEncoder = aimingMotor.getEncoder();

        // Tune PID values
        shooterMotorPID.setP(9e-3);
        shooterMotorPID.setI(1e-6);
        shooterMotorPID.setD(0);
        shooterMotorPID.setIZone(0);
        shooterMotorPID.setFF(0.000015);
        shooterMotorPID.setOutputRange(-1.0, 1.0);

        aimingMotorPID.setP(0.1);
        aimingMotorPID.setI(0);
        aimingMotorPID.setD(0);
        aimingMotorPID.setIZone(0);
        aimingMotorPID.setFF(0.000015);
        aimingMotorPID.setOutputRange(-1.0, 1.0);
    }

    public void shoot(double rpm) {
        shooterMotorPID.setReference(rpm, ControlType.kVelocity);
    }

    public void aimHeight(double rotations) {
        aimingMotorPID.setReference(rotations, ControlType.kPosition);
    }

    public void setShootingPercentOutput(double percentOutput) {
        shooterMotor.set(percentOutput);
    }

    public void setHoodPercentOutput(double percentOutput) {
        System.out.println(!lowerLimitSwitch.get());
        System.out.println(!upperLimitSwitch.get());
        if (getLowerLimitSwitchValue() && percentOutput > 0) {
            percentOutput = 0.0;
        } else if (getUpperLimitSwitchValue() && percentOutput < 0) {
            percentOutput = 0.0;
        }

        aimingMotor.set(percentOutput);
    }

    public int getShootingSpeed() {
        // Cast to int for ease of comparison
        return (int) shooterMotorEncoder.getVelocity();
    }

    public void setLowerLimitPosition(int position) {
        minHoodPosition = position;
        lowerLimitHit = true;
    }

    public void setUpperLimitPosition(int position) {
        maxHoodPosition = position;
        upperLimitHit = true;
    }

    public void setUpperLimitHit(boolean value) {
        upperLimitHit = value;
    }

    public void setLowerLimitHit(boolean value) {
        lowerLimitHit = value;
    }

    boolean isReadyToAim() {
        return readyToAim;
    }

    public void setReadyToAim(boolean readyToAim) {
        this.readyToAim = readyToAim;
    }

    public int getHoodPosition() {
        return (int) aimingMotorEncoder.getPosition();
    }

    public void setHoodPosition(double position) {
        position = MathUtil.clamp(position, -0.12d, 1.0d);

        position = minHoodPosition + (position * range);

        aimingMotorPID.setReference(position, ControlType.kPosition);
    }

    public boolean getUpperLimitSwitchValue() {
        return !upperLimitSwitch.get();
    }

    public boolean getLowerLimitSwitchValue() {
        return !lowerLimitSwitch.get();
    }

    public void hoodCalibration() {
        if (readyToAim) return;

        if (lowerLimitHit && upperLimitHit) {
            // Go back to center, ready to aim.
            readyToAim = true;
            range = maxHoodPosition - minHoodPosition;
            setHoodPosition(0.5);

            return;
        }

        // Approach the lower limit of the hood position.
        if (getLowerLimitSwitchValue() && !lowerLimitHit) {
            setLowerLimitPosition(getHoodPosition());

            // Approach the lower limit
            setHoodPercentOutput(-0.75);

            return;
        }

        // Approach the upper limit of the hood position.
        if (getUpperLimitSwitchValue() && !upperLimitHit) {
            setUpperLimitPosition(getHoodPosition());

            // Approach the upper limit
            setHoodPercentOutput(0.75);
        }
    }

    public void setPID(double p, double i, double d) {
        shooterMotorPID.setP(p);
        shooterMotorPID.setI(i);
        shooterMotorPID.setD(d);
    }

    public void setVisionLight(boolean visionLightEnabled) {
        visionLEDRing.set(visionLightEnabled);
    }
}