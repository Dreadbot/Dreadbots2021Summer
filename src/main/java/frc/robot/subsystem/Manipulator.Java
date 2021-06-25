package frc.robot.subsystem;

public class Manipulator {
	private Intake intake;
	private Feeder feeder;
	private Shooter shooter;

	private enum ShooterState { RAMPING, PUNCHING, RETRACTING, ADVANCE, ADVANCING };
	private enum GenevaState { MOVE, MOVING, STOPPED, FORWARD, BACKWARD };

	private GenevaState genevaState = GenevaState.STOPPED;
	private GenevaState genevaDirection = GenevaState.FORWARD;
	private ShooterState shooterState = ShooterState.RAMPING;
	private ShooterState lastShooterState;

	private int stateChangeCounter;
	private final int countsToExtend = 5;
	private int numPunches;

	public Manipulator(Intake intake, Feeder feeder, Shooter shooter){
		this.intake = intake;
		this.feeder = feeder;
		this.shooter = shooter;

		numPunches = 0;
		stateChangeCounter = 0;
	}

	public void prepareShot(double rpm, double aimPosition){
		shooter.shoot(rpm);
		shooter.setHoodPosition(aimPosition);
	}

	public double getSelectedRPM(double inches){
		inches /= 12;
		//equation from a regression we did by trial and error.
		double value = ((-0.0029 * inches * inches) + (0.188026 * inches) + 1.7676) * 1000 + 200;
		value = MathUtils.clamp((Double)value, 0d, 4550d);
		return value;
	}

	public double getSelectedHoodPosition(double inches){
		inches /= 12;
		//equation from a regression we did by trial and error
		return ((7.14e-7 * inches * inches) + (8.51e-4 * inches) + 0.398);
	}

	public int continuousShoot(double aimPosition, double genevaSpeed, double shootingRPM){
		// Finite state machine logic

		//Find difference between intended speed and actual speed
		int speedDifference = (int) (Math.abs(shooter.getShootingSpeed()) - shootingRPM);
		
		// If the speed is within the acceptable range for shooting, punch.
		if(shooterState == shooterStates.RAMPING && speedDifference < 200 && speedDifference > -25) {
			shooterState = shooterStates.PUNCHING;
		}
		// If punching and a tenth of a second has passed (stateChangeCounter) start to retract.
		else if(shooterState == shooterStates.PUNCHING && stateChangeCounter > countsToExtend) {
			shooterState = shooterStates.RETRACTING;
			stateChangeCounter = 0;
		}

		// If the punch is in a secure area, advance the Geneva drive.
		else if(shooterState == shooterStates.RETRACTING && feeder.getPunchSwitchState()) {
			shooterState = shooterStates.ADVANCE;
			stateChangeCounter = 0;
		}
		// If the Geneva is in transit, the state becomes "ADVANCING"
		else if(shooterState == shooterStates.ADVANCE && !feeder.getGenevaSwitchState()) {
			shooterState = shooterStates.ADVANCING;
		}
		// When the Geneva has reached the next punching area, begin ramping.
		else if(shooterState == shooterStates.ADVANCING && feeder.getGenevaSwitchState()) {
			shooterState = shooterStates.RAMPING;
		}

		//Choose behavior based on the FSM state
		switch(shooterState){
			case RAMPING:
				feeder.setSpin(0);
				break;
			case PUNCHING:
				feeder.setPunchExtension(true);
				stateChangeCounter++;
				break;
			case RETRACTING:
				if(lastShooterState != shooterStates.RETRACTING) {
					++numPunches;
				}
				feeder.setPunchExtension(false);
				break;
			case ADVANCE:
			case ADVANCING:
				feeder.setSpin(genevaSpeed);
				break;
		}
		
		shooter.setHoodPosition(aimPosition);
		shooter.shoot(-shootingRPM);

		lastShooterState = shooterState;
		return numPunches;
	}

	public void resetManipulatorElements(){
		//This function should be called continuously if the system is not shooting or collecting power cells
		// this function will get the system back into a state where the punch is retracted and the geneva gear is aligned in order to be able to begin the FSM again
		numPunches = 0;

		//If the punch is out, retract it
		if(feeder.getPunchExtension()){
			feeder.setPunchExtension(false);
		}
		//Then, once its retracted, if the geneva gear isn't at the limit switch, turn it slowly
		else if(!feeder.getPunchExtension() && !feeder.getGenevaSwitchState()){
			//This is commented out in the C++ original and I'm not sure why
			//feeder.setSpin(0.2);
		}
		//Once it reaches the limit switch, stop it
		else{
			feeder.setSpin(0);
		}
		shooterState = shooterStates.RAMPING;
		if(shooter.getAimReadiness()){
			shooter.setHoodPosition(0);
		}
		shooter.setShootingPercentOutput(0);
		sensorAdvanceGeneva(false, false);
	}

	public int getNumPunches(){
		return numPunches;
	}

	public void sensorAdvanceGeneva(boolean spin, boolean forward){
		double genevaSpeed = 0.5;
		if(genevaState == genevaStates.STOPPED && spin){
			if(forward){
				feeder.setSpin(-genevaSpeed);
				genevaDirection = genevaStates.FORWARD;
			}
			else{
				feeder.setSpin(genevaSpeed);
				genevaDirection = genevaStates.BACKWARD;
			}
			genevaState = genevaStates.MOVE;
		}
		else if(genevaState == genevaStates.MOVE && !feeder.getGenevaSwitchState()){
			genevaState = genevaStates.MOVING;
		}
		else if(genevaState == genevaStates.MOVING && feeder.getGenevaSwitchState()){
			feeder.setSpin(0);
			genevaState = genevaStates.STOPPED;
		}

		if(genevaState == genevaStates.MOVE || genevaState == genevaStates.MOVING){
			if(genevaDirection == genevaStates.FORWARD){
				feeder.setSpin(-genevaSpeed);
			}
			else{
				feeder.setSpin(genevaSpeed);
			}
		}
	}

	public void genevaSetSpin(double power){
		feeder.setSpin(power);
	}

	public int getSensorAdvanceGenevaState(){
		return genevaState.ordinal();
	}

	
	public Intake getIntake(){
		return intake;
	}
	public Feeder getFeeder(){
		return feeder;
	}
	public Shooter getShooter(){
		return shooter;
	}

}