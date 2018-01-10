package org.usfirst.frc.team2791.swerve;

import org.usfirst.frc.team2791.util.ShakerGamepad;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.Joystick;

/**
 * Calculates the angle and speed of each 4 wheels based
 * on the input from two 2-axis joysticks.</br>
 * Kinematics from <a href = https://www.chiefdelphi.com/media/papers/2426)> Ether</a>
 * 
 * @author created by Unnas Hussain on 8/2/2017
 */
public class SwerveHelper {

	public final static double WHEELBASE_LENGTH = 36.0/12.0;  //based on 2791's robot Stoker from FRC SteamWorks
	public final static double WHEELBASE_WIDTH = 28.0/12.0;   //based on 2791's robot Stoker from FRC SteamWorks

	/*
	 * Math Conventions:
	 * 
	 * 
	 * Wheel ID Numbers:
	 * 0 = front_right;
	 * 1 = front_left;
	 * 2 = rear_left;
	 * 3 = rear_right;
	 * 
	 * 0.0 degrees is the wheels facing the front of the bot
	 * 
	 */
	
	
	protected static double[] wheelSpeed = new double[4];
	protected static double[] wheelAngle = new double[4];

	protected static GyroBase m_gyro = null;

	public static boolean fieldCentric = true;

	public static boolean useAngleToReverse = true;


	public static double getSpeedValue(Joystick drive, Joystick rotate, int wheelID){
		calculate(drive.getY(), drive.getX(), rotate.getX());
		return wheelSpeed[wheelID];
	}

	public static double getAngleValue(Joystick drive, Joystick rotate,  int wheelID){
		calculate(drive.getY(), drive.getX(), rotate.getX());
		return wheelAngle[wheelID];
	}	

	public static double getSpeedValue(ShakerGamepad input, int wheelID){
		calculate(input.getSwerveDriveSpeed(), input.getSwerveDriveStrafe(), input.getSwerveDriveRotation());
		return wheelSpeed[wheelID];
	}

	public static double getAngleValue(ShakerGamepad input,  int wheelID){
		calculate(input.getSwerveDriveSpeed(), input.getSwerveDriveStrafe(), input.getSwerveDriveRotation());
		return wheelAngle[wheelID];
	}	

	public static void calculate(double forward, double strafe, double rotate){

		if(fieldCentric && getGyro() != null){
			double gyroAngle =Math.toRadians(getGyro().getAngle());

			double temp = forward * Math.cos(gyroAngle) + strafe*Math.sin(gyroAngle);
			strafe = -forward*Math.sin(gyroAngle) + strafe*Math.cos(gyroAngle);
			forward = temp;
		}else if (fieldCentric && getGyro() == null) {
			System.out.println ("No Gyro found, Swerve calculations will be Robot Centric");
		}

		/*
		 * Inverse Kinematics Based on work from Ether on Cheif Delphi
		 */
		double frontRightX,frontRightY,backLeftX,backLeftY;
		double L = WHEELBASE_LENGTH, W = WHEELBASE_WIDTH;
		double R = Math.sqrt(Math.pow(L, 2) + Math.pow(W, 2) );

		frontRightX = strafe - rotate*(L/R);
		frontRightY = strafe + rotate*(L/R);
		backLeftX = forward - rotate*(W/R);
		backLeftY = forward + rotate*(W/R);

		wheelSpeed[0] = Math.sqrt((Math.pow(frontRightY,2) + Math.pow(backLeftX,2)));
		wheelSpeed[1] = Math.sqrt((Math.pow(frontRightY,2) + Math.pow(backLeftY,2)));
		wheelSpeed[2] = Math.sqrt((Math.pow(frontRightX,2) + Math.pow(backLeftY,2)));
		wheelSpeed[3] = Math.sqrt((Math.pow(frontRightX,2) + Math.pow(backLeftX,2)));
		normalizeSpeeds();

		wheelAngle[0] = Math.atan2(frontRightY, backLeftX) * 180/Math.PI;
		wheelAngle[1] = Math.atan2(frontRightY, backLeftY) * 180/Math.PI;
		wheelAngle[2] = Math.atan2(frontRightX, backLeftY) * 180/Math.PI;
		wheelAngle[3] = Math.atan2(frontRightX, backLeftX) * 180/Math.PI;

		if(!useAngleToReverse)
			reverseWithSpeed();

	}

	/**
	 * The kinematics create backwards vectors by rotating the wheels and always driving in a positive direction,
	 * (So going backwards would be Speed == 1.0, Angle == 180) </br>
	 * 
	 * If the vehicle design is such that it would better to avoid rotating the modules,
	 * use this method after the kinematics are done.
	 * (Then going backwards will be Speed ==-1.0, Angle == 0)
	 */
	private static void reverseWithSpeed(){
		double temp_wheelAngle[] = wheelAngle;

		if(temp_wheelAngle[0] > 90.0 || temp_wheelAngle[0] < -90.0){
			wheelAngle[0] = temp_wheelAngle[0] - Math.copySign(180.0, temp_wheelAngle[0]);
			wheelSpeed[0] *= -1.0;
		}

		if(temp_wheelAngle[1] > 90.0 || temp_wheelAngle[1] < -90.0){
			wheelAngle[1] = temp_wheelAngle[1] - Math.copySign(180.0, temp_wheelAngle[1]);
			wheelSpeed[1] *= -1.0;
		}

		if(temp_wheelAngle[2] > 90.0 || temp_wheelAngle[2] < -90.0){
			wheelAngle[2] = temp_wheelAngle[2] - Math.copySign(180.0, temp_wheelAngle[2]);
			wheelSpeed[2] *= -1.0;
		}

		if(temp_wheelAngle[3] > 90.0 || temp_wheelAngle[3] < -90.0){
			wheelAngle[3] = temp_wheelAngle[3] - Math.copySign(180.0, temp_wheelAngle[0]);
			wheelSpeed[3] *= -1.0;
		}
	}


	/**Normalize speeds to be in a range from 0 to +1.0*/
	private static void normalizeSpeeds(){
		double max = Math.max(wheelSpeed[0], wheelSpeed[1]);
		max = Math.max(max, wheelSpeed[2]);
		max = Math.max(max, wheelSpeed[3]);

		if(max>1){
			wheelSpeed[0] /=max;
			wheelSpeed[1] /=max;
			wheelSpeed[2] /=max;
			wheelSpeed[3] /=max;
		}
	}

	public static void setToFieldCentric(){
		fieldCentric = true;
	}

	public static void setToBotCentric(){
		fieldCentric = false;
	}

	
	/**
	 * Reversing with Rotation means that when you go backwards,
	 * the speed will be in a positive direction but the angle will spin.
	 * This means that the speed is never negative </br>
	 * (So going backwards would be Speed == 1.0 , Angle == 180)
	 */
	public static void setReversingToRotation(){
		useAngleToReverse = true;
	}

	/**
	 * Reversing with Speed means that when you go backwards,
	 * the speed will be in a negative direction and there will be less rotation.
	 * This means that the angle is never outside the range -90 deg to 90 deg </br>
	 * (So going backwards would be Speed == -1.0 , Angle == 0)
	 */
	public static void setReversingToSpeed(){
		useAngleToReverse = false;
	}

	public static void setGyro(GyroBase gyro){
		m_gyro = gyro;
		fieldCentric = true;
	}

	public static GyroBase getGyro(){
		return m_gyro;
	}

}

