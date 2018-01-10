package org.usfirst.frc.team2791.swerve;

import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class TalonSRXModule extends ControlModule {
	
	private TalonSRX m_rotation; 
	private TalonSRX m_wheel;
	
	private double wheelDiameter = 4.0;
	private double rotationDiameter = 4.0;
	private int ticksPerRev = 4096;
	
	public TalonSRXModule(TalonSRX rotation, TalonSRX wheel, WheelPosition pos) {
		super(pos);
		m_rotation = rotation;
		m_wheel = wheel;
	}
	
	public TalonSRXModule(TalonSRX rotation, TalonSRX wheel, WheelPosition pos, double wheelDiam, double rotationDiam, int ticksPerRev) {
		this(rotation, wheel, pos);
		setWheelDiameter(wheelDiam);
		setRotationDiameter(rotationDiam);
		setTicksPerRev(ticksPerRev);
	}
	
	@Override
	public void setSpeedAndAngle(Joystick drive, Joystick rotate){
		m_wheel.set(ControlMode.PercentOutput, SwerveHelper.getSpeedValue(drive, rotate, position.wheelNumber));
		m_rotation.set(ControlMode.Position, SwerveHelper.getAngleValue(drive, rotate, position.wheelNumber));
	}
	
	@Override
	public void setRotationPID(double kp, double ki, double kd){
		m_rotation.config_kP(0, kp, 0);
		m_rotation.config_kI(1, ki, 0);
		m_rotation.config_kD(2, kd, 0);
	}

	@Override
	public void setRotationPIDF(double kp, double ki, double kd, double kf){
		m_rotation.config_kP(0, kp, 0);
		m_rotation.config_kI(1, ki, 0);
		m_rotation.config_kD(2, kd, 0);
		m_rotation.config_kF(3, kf, 0);	}
	
	@Override
	public double getRotationP(){
		return m_rotation.configGetParameter(0, 0, 0);
	}

	@Override
	public double getRotationI(){
		return m_rotation.configGetParameter(1, 0, 0);
	}

	@Override
	public double getRotationD(){
		return m_rotation.configGetParameter(2, 0, 0);
	}

	@Override
	public double getRotationF(){
		return m_rotation.configGetParameter(3, 0, 0);
	}
	
	@Override
	public double getAngle(){
		return (m_rotation.getSelectedSensorPosition(0) *  Math.PI * wheelDiameter);
	}
	
	@Override
	public double getDistance(){
		return (m_wheel.getSelectedSensorPosition(0) *  Math.PI * wheelDiameter) / ticksPerRev;
	}

	@Override
	public double getVelocity() {
		return m_wheel.getSelectedSensorVelocity(0) * 600/ticksPerRev;
	}

	@Override
	public double getAcceleration() {
		return super.getAcceleration();
	}

	public double getWheelDiameter() {
		return wheelDiameter;
	}

	public void setWheelDiameter(double wheelDiameter) {
		this.wheelDiameter = wheelDiameter;
	}

	public double getRotationDiameter() {
		return rotationDiameter;
	}

	public void setRotationDiameter(double rotationDiameter) {
		this.rotationDiameter = rotationDiameter;
	}

	public int getTicksPerRev() {
		return ticksPerRev;
	}

	public void setTicksPerRev(int ticksPerRev) {
		this.ticksPerRev = ticksPerRev;
	}	

}
