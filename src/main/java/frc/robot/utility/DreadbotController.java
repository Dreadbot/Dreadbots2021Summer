package frc.robot.utility;

import edu.wpi.first.wpilibj.Joystick;

/**
 * A custom Facade pattern for the default WPI Joystick class
 * designed for simplicity of use.
 * This facade has only been tested on the Logitech F310 Gamepad.
 */
public final class DreadbotController {
    private final Joystick joystick;


    /**
     * Constructs a DreadbotController object with the ID of the gamepad HID, which is
     * passed to the WPI Joystick class underneath.
     *
     * @param joystickPort The port on the Driver Station that the joystick is plugged into.
     */
    public DreadbotController(int joystickPort) {
        this.joystick = new Joystick(joystickPort);
    }

    /*
    * Controller bindings
    * Controller 1
    *   A-
    *   B-
    *   X-
    *   Y-
    *   RB-Pull ball intake up
    *   RT-
    *   LB-
    *   LT-
    *   Back-
    *   Start-
    *   Dpad up-
    *   Dpad left-
    *   Dpad right- 
    *   Dpad down-
    *   Right stick-Turn robot
    *   Left stick- Drive Robot
    * Controller 2
    *   A-Intake pull ball in
    *   B- Hold to shoot
    *   X-Intake push ball out
    *   Y- Hold to shoot 
    *   RB-Advance geneva
    *   RT-move climber winch down
    *   LB-Reverse geneva
    *   LT-Move climber winch up
    *   Back-Put climber down
    *   Start- Put Climber up
    *   Dpad up-
    *   Dpad left-
    *   Dpad right- 
    *   Dpad down-
    *   Right stick-
    *   Left stick-
    **/ 
    /**
     * @return The value of the side-to-side motion on the left joystick.
     */
    @SuppressWarnings("unused")
    public double getXAxis() {
        return joystick.getRawAxis(0);
    }

    /**
     * @return The value of the forward/backward motion on the left joystick.
     */
    @SuppressWarnings("unused")
    public double getYAxis() {
        return joystick.getRawAxis(1);
    }

    /**
     * @return The value of the side-to-side motion on the right joystick.
     */
    @SuppressWarnings("unused")
    public double getZAxis() {
        return joystick.getRawAxis(2);
    }

    /**
     * @return The value of the forward/backward motion on the right joystick.
     */
    @SuppressWarnings("unused")
    public double getWAxis() {
        return joystick.getRawAxis(3);
    }

    /**
     * @return The state of the blue 'X' button on the right side of the gamepad.
     */
    @SuppressWarnings("unused")
    public boolean isXButtonPressed() {
        return joystick.getRawButton(1);
    }

    /**
     * @return The state of the green 'A' button on the right side of the gamepad.
     */
    @SuppressWarnings("unused")
    public boolean isAButtonPressed() {
        return joystick.getRawButton(2);
    }

    /**
     * @return The state of the red 'B' button on the right side of the gamepad.
     */
    @SuppressWarnings("unused")
    public boolean isBButtonPressed() {
        return joystick.getRawButton(3);
    }

    /**
     * @return The state of the orange 'Y' button on the right side of the gamepad.
     */
    @SuppressWarnings("unused")
    public boolean isYButtonPressed() {
        return joystick.getRawButton(4);
    }

    /**
     * @return The state of the left flat button on the front face of the gamepad.
     */
    @SuppressWarnings("unused")
    public boolean isLeftBumperPressed() {
        return joystick.getRawButton(5);
    }

    /**
     * @return The state of the right flat button on the front face of the gamepad.
     */
    @SuppressWarnings("unused")
    public boolean isRightBumperPressed() {
        return joystick.getRawButton(6);
    }

    /**
     * @return The state of the left trigger on the front face of the gamepad.
     */
    @SuppressWarnings("unused")
    public boolean isLeftTriggerPressed() {
        return joystick.getRawButton(7);
    }

    /**
     * @return The state of the right trigger on the front face of the gamepad.
     */
    @SuppressWarnings("unused")
    public boolean isRightTriggerPressed() {
        return joystick.getRawButton(8);
    }

    /**
     * @return The state of the 'BACK' utility button in the center of the gamepad.
     */
    @SuppressWarnings("unused")
    public boolean isBackButtonPressed() {
        return joystick.getRawButton(9);
    }

    /**
     * @return The state of the 'START' utility button in the center of the gamepad.
     */
    @SuppressWarnings("unused")
    public boolean isStartButtonPressed() {
        return joystick.getRawButton(10);
    }

    /**
     * Returns the lower level edu.wpi.first.wpilibj.Joystick class used by this
     * facade pattern.
     *
     * @return The native WPI object used to construct this Facade pattern with.
     */
    @SuppressWarnings("unused")
    public Joystick getNativeWPIJoystick() {
        return joystick;
    }

    /**
     * Delegate method to the getRawButton(int) method of the Joystick class.
     *
     * @param button The ID of the button on the gamepad.
     * @return The state of the given button.
     */
    @SuppressWarnings("unused")
    public boolean getRawButton(int button) {
        return joystick.getRawButton(button);
    }

    /**
     * Delegate method to the getRawAxis(int) method of the Joystick class.
     *
     * @param axis The ID of the axis on the gamepad.
     * @return The current position of the given axis.
     */
    @SuppressWarnings("unused")
    public double getRawAxis(int axis) {
        return joystick.getRawAxis(axis);
    }

    /**
     * Delegate method to the isConnected(int) method of the Joystick class.
     *
     * @return Whether the gamepad is connected or not.
     */
    @SuppressWarnings("unused")
    public boolean isConnected() {
        return joystick.isConnected();
    }

    /**
     * Delegate method to the getName() method of the Joystick class.
     *
     * @return Name of the gamepad HID.
     */
    @SuppressWarnings("unused")
    public String getName() {
        return joystick.getName();
    }
}
