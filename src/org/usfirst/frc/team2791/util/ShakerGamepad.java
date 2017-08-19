package org.usfirst.frc.team2791.util;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Shaker's Custom class for mapping gamepad joysticks
 * 
 * Uses WPILib's open source Joystick library, 
 * but accounts for deadzones near the center of joystick controllers. </br>
 * Added for this library: Logic to translate joystick values for SwerveDrive (these start at line 62)
 * 
 * @author Created by Akhil Jacob on 4/10/2016.
 * @author Swerve Logic Added by Unnas Hussain on 8/2/2017
 */
public class ShakerGamepad extends Joystick {
	
	/* in the middle of the joystick, there may be a deadzone 
	 * where the joystick value doesn't change or is inaccurate. 
	 * This value is the max joystick value where that does not occur
	 */
	public static final double DEADZONE = 0.08; 


	public ShakerGamepad(int port) {
		super(port);
	}

	public double getAxisLeftX() {
		return deadzone(DEADZONE, super.getRawAxis(0), 1.0);
	}

	public double getAxisLeftY() {
		return deadzone(DEADZONE, super.getRawAxis(1), 1.0);
	}

	public double getAxisRightX() {
		return deadzone(DEADZONE, super.getRawAxis(4), 1.0);
	}

	public double getAxisRightY() {
		return deadzone(DEADZONE, super.getRawAxis(5), 1.0);
	}

	/**
	 * adjust the value returned by a joystick on a controller
	 * while accounting for the deadzone
	 */
	public static double deadzone(double min, double val, double max) {
		double absVal = Math.abs(val);
		double absMin = Math.abs(min);
		double absMax = Math.abs(max);

		if (absMin <= absVal && absVal <= absMax) {
			return val;
		} else if (absVal <= absMin) {
			return 0.0;
		} else {
			return val < 0 ? -absMax : absMax;
		}
	}

	//*****************Swerve Drive Methods*********************
	//These are required in any Gampepad Class that you choose to use if you don't want to use ShakerGamepad


	/*
	 * Uses the Left Joystick for Driving/Strafing
	 * and the right Joystick for Rotating
	 */

	public double getSwerveDriveStrafe() {
		// Does the math to get a Swerve Drive strafe from the left joystick X
		double leftAxis;
		if (getAxisLeftX() < 0)
			leftAxis = -Math.pow(getAxisLeftX(), 2);
		else
			leftAxis = Math.pow(getAxisLeftX(), 2);
		return leftAxis;
	} 

	public double getSwerveDriveRotation() {
		// Does the math to get a Swerve Drive rotation from the right joystick X
		double rightAxis;
		if (getAxisRightX() < 0)
			rightAxis = -Math.pow(getAxisRightX(), 2);
		else
			rightAxis = Math.pow(getAxisRightX(), 2);
		return rightAxis;
	}

	public double getSwerveDriveSpeed() {
		// Does the math to get a Swerve Drive speed from the left joystick Y
		double leftAxis;
		if (getAxisLeftY() < 0)
			leftAxis = -Math.pow(getAxisLeftY(), 2);
		else
			leftAxis = Math.pow(getAxisLeftY(), 2);
		return leftAxis;
	}



}