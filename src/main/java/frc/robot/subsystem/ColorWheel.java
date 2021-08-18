package frc.robot.subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utility.DreadbotController;
import com.revrobotics.ColorSensorV3;
import frc.robot.utility.DreadbotConstants;
import edu.wpi.first.wpilibj.I2C;

public class ColorWheel {
    //Hardware
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    public Solenoid colorWheelExtension = new Solenoid(DreadbotConstants.COLOR_WHEEL_SOLENOID_ID);
    private CANSparkMax colorWheelManipulator = new CANSparkMax(DreadbotConstants.COLOR_WHEEL_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    private ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    //variables
    private enum WheelState { NotSpinning, InitSpinning, Spinning}
    private WheelState currentSpinState;
    private Color currentColor;
    private ColorMatchResult colorMatch;
    private int numberOfSpins;
    private int numberOfColorSamples;
    private boolean isTargetColorRed;
    private double currentColorConfidence;
    private int currentButtonSetting;
    private final double colorConfidenceSetting = 0.9;
    private Color colorToMatch;
    private ColorMatch colorMatcher = new ColorMatch();

    
    public ColorWheel() {
        
        
        colorMatcher.addColorMatch(DreadbotConstants.kBlueTarget);
        colorMatcher.addColorMatch(DreadbotConstants.kGreenTarget);
        colorMatcher.addColorMatch(DreadbotConstants.kRedTarget);
        colorMatcher.addColorMatch(DreadbotConstants.kYellowTarget);
    }

    //Update RotateToNumber to not take in sensor and get current color from m_colorMatch
    /*public void RotateToNumber(){
    SmartDashboard.putNumber("Number spins", numberOfSpins);

    SmartDashboard.putString("Spin State", currentSpinState.toString());

        if(currentSpinState == WheelState.NotSpinning){
            currentSpinState = WheelState.InitSpinning;
        }
        else if ( currentSpinState == WheelState.InitSpinning){
            numberOfSpins = 0;
            colorWheelManipulator.set(0.7);
            currentSpinState = WheelState.Spinning;
        }
        else if (currentSpinState == WheelState.Spinning){
            if(numberOfSpins > 7 ){ // colors show up twice, so 7 rotations = 3.5 spins
                colorWheelManipulator.set(0);
                return;
            } 
            //A value between 0 and 1, 1 being absolute perfect color match
            currentColorConfidence = 0;
            currentColor =  colorSensor.getColor();
            colorMatch = colorMatcher.matchClosestColor(currentColor);

            PrintColor(currentColor, currentColorConfidence);

            if(colorMatch.color == DreadbotConstants.kRedTarget &&
            !isTargetColorRed && 
            currentColorConfidence>= colorConfidenceSetting){
                numberOfSpins++;
                isTargetColorRed = true;
            }
            else if (!(DreadbotConstants.kRedTarget == colorMatch.color)){
                isTargetColorRed = false;
            }
        }
    }*/ 
    // Code wasn't used, keep until sure its not used
    public void SetRotationState(WheelState rotationState){
        currentSpinState = rotationState;
    }

    public void RotateToColor(Color targetColor){
        currentColor = colorSensor.getColor();
        colorMatch = colorMatcher.matchClosestColor(currentColor);
        currentColorConfidence = colorMatch.confidence;

        //SmartDashboard.putNumber("Spin State", currentSpinState);

        if(currentSpinState == WheelState.NotSpinning /*&& secondaryJoystick.isBackButtonPressed()*/){
            currentSpinState = WheelState.InitSpinning;
            currentButtonSetting = 2;
        }
        if(currentSpinState == WheelState.InitSpinning && currentButtonSetting == 2){
            currentSpinState = WheelState.Spinning;
            colorWheelManipulator.set(0.2);
        }
        if(currentSpinState == WheelState.Spinning && currentButtonSetting == 2){
            PrintColor(colorMatch.color, currentColorConfidence);

            if(colorMatch.color == targetColor && currentColorConfidence>= colorConfidenceSetting){
                if(numberOfColorSamples > 5){
                    currentSpinState = WheelState.NotSpinning;

                    colorWheelManipulator.set(0);

                    numberOfColorSamples = 0;
                    currentButtonSetting = 0;
                }
                else{
                    numberOfColorSamples +=1;
                }
            }
        }
    }

    public void PrintColor(Color color, double colorConfidence){
        if(color == DreadbotConstants.kBlueTarget){
            SmartDashboard.putString("Color:", "Blue");
            System.out.println("Blue\n" + colorConfidence);
        }
        else if(color == DreadbotConstants.kRedTarget){
            SmartDashboard.putString("Color:", "Red");
            System.out.println("Red\n"+ colorConfidence);
        }
        else if(color == DreadbotConstants.kYellowTarget){
            SmartDashboard.putString("Color:", "Yellow");
            System.out.println("Yellow\n"+ colorConfidence);
        }
        else if(color == DreadbotConstants.kGreenTarget){
            SmartDashboard.putString("Color:", "Green");
            System.out.println("Green\n"+ colorConfidence);
        }
        else{
            SmartDashboard.putString("Color:", "no color detected");
            System.out.println("No color detected");
        }

        SmartDashboard.putNumber("Confidence", colorConfidence);
    }

    public Color ColorDataGetter(){
        String colorData;
        colorData = DriverStation.getInstance().getGameSpecificMessage();
        if(colorData.length()>0){
            SmartDashboard.putString("Color:", colorData);
            switch (colorData.charAt(0)){
                case 'B':
                    colorToMatch =  Color.kBlue;
                    break;
                case 'G':
                    colorToMatch =  Color.kGreen;
                    break;
                case 'R':
                    colorToMatch =  Color.kRed;
                    break;
                case 'Y':
                    colorToMatch =  Color.kYellow;
                    break;
                default:
                SmartDashboard.putString("Color data is not recongized, color provided:", colorData);
                break;
            }
            return colorToMatch; 
        }
        else {return null;}
    }
}
