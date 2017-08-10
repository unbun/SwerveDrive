package team2791;

import java.util.Scanner;

import org.usfirst.frc.team2791.swerve.SwerveHelper;

/**
 * A simple class to quickly test the Swerve math.
 * Represents the Joystick Values with doubles recieved from the scanner
 * @author Unnas
 *
 */
public class Test {
	
    public final static double WHEELBASE_LENGTH = 36.0/12.0; 
    public final static double WHEELBASE_WIDTH = 28.0/12.0;
	
	protected static double[] wheelSpeed = new double[4];
	protected static double[] wheelAngle = new double[4];

	public static boolean useAngleToReverse = true;
	
	public static void main(String args[]){
		
		Scanner sc = new Scanner(System.in);
		double rx,ry,lx,ly, gyro;

		while(true){
			System.out.print("Enter RX:");
			rx = sc.nextDouble();
			System.out.print("Enter LX:");
			lx = sc.nextDouble();
			System.out.print("Enter LY:");
			ly = sc.nextDouble();
			System.out.print("Enter Gyro Angle ( - 2791 to void field centricty)");
			gyro = sc.nextDouble();
			
			getSwerveMath(rx,lx,ly,gyro);
			
			System.out.print("Wheel 1 Speed: " + wheelSpeed[0] +" Angle: " + wheelAngle[0]);
			sc.nextLine();
			sc.nextLine();

			System.out.print("Wheel 2 Speed: " + wheelSpeed[1] +" Angle: " + wheelAngle[1]);
			sc.nextLine();

			System.out.print("Wheel 3 Speed: " + wheelSpeed[2] +" Angle: " + wheelAngle[2]);
			sc.nextLine();

			System.out.print("Wheel 4 Speed: " + wheelSpeed[3] +" Angle: " + wheelAngle[3]);
			sc.nextLine();

			
		}
	}
	
	/**
	 * This method is copied from SwerveMath.java.
	 *  I can't use the method in SwerveMath.java because it requires actual joysticks and gyros, 
	 *  but I am just using the Scanner and doubles to replace those
	 *  ... i'm lazy and just want to know if the math works 0:)
	 */
	public static void getSwerveMath(double rightX, double leftX, double leftY, double gyro){
		

		double forward = leftY;
		double strafe = leftX;
		double rotate = rightX;		
		double gyroAngle = Math.toRadians(gyro);
		
		if(gyroAngle == -2791){
			SwerveHelper.setToBotCentric();
			SwerveHelper.calculate(forward, strafe, rotate);
		}
		
		else if (gyroAngle != -2791){ //if testing field centricity, then because there's no gyro, that has to be done locally
			double temp = forward * Math.cos(gyroAngle) + strafe*Math.sin(gyroAngle);
			strafe = -forward*Math.sin(gyroAngle) + strafe*Math.cos(gyroAngle);
			forward = temp;
		}

		//simplified variables for inverse kinematics
		double A,B,C,D;
		double L = WHEELBASE_LENGTH, W = WHEELBASE_WIDTH;
		double R = Math.sqrt(Math.pow(L, 2) + Math.pow(W, 2) );

		A = strafe - rotate*(L/R);
		B = strafe + rotate*(L/R);
		C = forward - rotate*(W/R);
		D = forward + rotate*(W/R);

		wheelSpeed[0] = Math.sqrt((Math.pow(B,2) + Math.pow(C,2)));
		wheelSpeed[1] = Math.sqrt((Math.pow(B,2) + Math.pow(D,2)));
		wheelSpeed[2] = Math.sqrt((Math.pow(A,2) + Math.pow(D,2)));
		wheelSpeed[3] = Math.sqrt((Math.pow(A,2) + Math.pow(C,2)));
		normalizeSpeeds();

		wheelAngle[0] = Math.atan2(B, C) * 180/Math.PI;
		wheelAngle[1] = Math.atan2(B, D) * 180/Math.PI;
		wheelAngle[2] = Math.atan2(A, D) * 180/Math.PI;
		wheelAngle[3] = Math.atan2(A, C) * 180/Math.PI;
		
		if(!useAngleToReverse)
			reverseWithSpeed();

	}
	
private static void reverseWithSpeed(){
		
//		System.out.println("REVERSING WITH SPEED");
		
		if(wheelAngle[0] > 90.0 || wheelAngle[0] < -90.0){
			wheelAngle[0] = wheelAngle[0] - Math.copySign(180.0, wheelAngle[0]);
			wheelSpeed[0] *= -1.0;
		}
		
		if(wheelAngle[1] > 90.0 || wheelAngle[1] < -90.0){
			wheelAngle[1] = wheelAngle[1] - Math.copySign(180.0, wheelAngle[1]);
			wheelSpeed[1] *= -1.0;
		}
		
		if(wheelAngle[2] > 90.0 || wheelAngle[2] < -90.0){
			wheelAngle[2] = wheelAngle[2] - Math.copySign(180.0, wheelAngle[2]);
			wheelSpeed[2] *= -1.0;
		}
		
		if(wheelAngle[3] > 90.0 || wheelAngle[3] < -90.0){
			wheelAngle[3] = wheelAngle[3] - Math.copySign(180.0, wheelAngle[0]);
			wheelSpeed[3] *= -1.0;
		}
	}
	
	
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
}
