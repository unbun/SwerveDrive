package org.usfirst.frc.team2791.swerve;


import org.usfirst.frc.team2791.util.BasicPID;
import org.usfirst.frc.team2791.util.ShakerGamepad;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;

/**
 * A TalonModule allows you to easily control the speed controllers for 
 * both rotation and speed</br>
 * The rotation speed controller has a PIDController with an potentiometer source. 
 * <i>(TODO: setup an option to use absolute encoders)</i></br>
 * 
 * @author created by Unnas Hussain on 8/2/2017
 */
public class TalonModule{
		
	public int wheelTag;
	
	private Talon rotation; //if you aren't using CTRE Talons and are having calibrations, try using the PWMSpeedController object instead
	private Talon wheel; //if you aren't using CTRE Talons and are having calibrations, try using the PWMSpeedController object instead

	private BasicPID rotationPID;
	public double rotateP = 1.0;
	public double rotateI = 0.0;
	public double rotateD = 0.0;

	private AnalogPotentiometer absoluteAngleEncoder;
	private Encoder speedEncoder = null;

	private double previousRate = 0;
	private double previousTime = 0;
	private double filteredAccel = 0;

	/**
	 * 
	 * location determines which module is which
	 * @param location 0 = front right / 1 = front left / 2 = back left / 3 = back right
	 */
	public TalonModule(int rotationPort, int wheelPort, int potentiometerPort, int location){
		wheelTag = location;
		
		rotation = new Talon(rotationPort);
		wheel = new Talon(wheelPort);
		absoluteAngleEncoder = new AnalogPotentiometer(potentiometerPort, 360.0, 0);

		rotationPID = new BasicPID(rotateP, rotateI, rotateD);
		
	}
	
	/**
	 * 
	 * location determines which module is which
	 * @param location 0 = front right / 1 = front left / 2 = back left / 3 = back right
	 */
	public TalonModule(int rotationPort, int wheelPort, int encoderPort, int wheelEncoderA, int wheelEncoderB, int location){
		this(rotationPort, wheelPort, encoderPort, location);
		speedEncoder = new Encoder(wheelEncoderA, wheelEncoderB);
	}

	public void setSpeedAndAngle(Joystick drive, Joystick rotate){
		wheel.set(SwerveHelper.getSpeedValue(drive, rotate, wheelTag));
		rotationPID.updateAndGetOutput(SwerveHelper.getAngleValue(drive, rotate, wheelTag));
	}
	
	public void setSpeedAndAngle(ShakerGamepad driver){
		wheel.set(SwerveHelper.getSpeedValue(driver, wheelTag));
		rotationPID.updateAndGetOutput(SwerveHelper.getAngleValue(driver, wheelTag));
	}

	//*********************PID Helper Methods******************//


	public void resetSpeedEncoder(){
		if(speedEncoder != null)
			this.speedEncoder.reset();
	}
	//************** Pos/Vel/Acc Helper Methods **************// 
	
	public double getAngle(){
		return absoluteAngleEncoder.get();
	}

	public double getDistance() {
		if(speedEncoder != null)
			return speedEncoder.getDistance();
		return 0.0;
	}

	public double getVelocity() {
		if(speedEncoder != null)
			return speedEncoder.getRate();
		return 0.0;

	}

	public double getRightAcceleration() {

		double currentRate = getVelocity();
		double currentTime = Timer.getFPGATimestamp();

		double acceleration = (currentRate - previousRate) / (currentTime - previousTime);

		previousRate = currentRate;
		previousTime = currentTime;

		filteredAccel = acceleration * 0.5 + filteredAccel * 0.5; //dampens random spikes due to derivations

		return filteredAccel;
	}	


}
