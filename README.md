# SwerveDrive

A simple swerve drive controller for 4 wheel holonomic FRC robots. This was written for FRC 2791. 

This uses FRC2791's custom two 2-axis Joystick Class (ShakerGamepad.java) as controllers.

This was designed to be dropped into whatever structure your team is comfortable using, which is why it's just a helper, not a full drivetrain subsystem. 


## How to use:


### Step 1: Create a Drivetrain with TalonModules

In your Drivetrain subsystem, initalize 4 TalonModules with the PWM ports for each motor, the rotation encoder, and which wheel you are creating (<i>FRONT_RIGHT,FRONT_LEFT,BACK_LEFT,BACK_RIGHT</i>). 

```java
TalonModule wheel1 =  new TalonModule(rotationTalonPort, speedTalonPort,
  potentiometerPort, WheelTag.FRONT_RIGHT);
```

You can also add an encoder for wheels:

```java
TalonModule wheel1 =  new TalonModule(rotationTalonPort, speedTalonPort, 
  rotationPotentiometerPort, speedEncoderPortA, speedEncoderPortB, WheelTag.FRONT_RIGHT);

//OR

wheel1.setEncoder(speedEncoderPortA, speedEncoderPortB);
```


### Step 2: Configure your SwerveHelper.

#### 2a) Send your drivetrain's gyro to Swervehelper (neccessary if you want field centricty).</br>

```java
//setting a non-null gyro will automatically set the helper to field-centric)
SwerveHelper.setGyro(drivetrain.getGyro());
```

#### 2b) Set up other options, like bot centricity or you can change reversing methods <i>(more info on what each reversing method is available in the comments)</i>

```java
SwerveHelper.setReversingToRotation();
//OR
SwerveHelper.setReversingToSpeed();
```

</br>

```java
SwerveHelper.setToBotCentric();
//OR
SwerveHelper.setToFieldCentric();
```


### Step 3: Make the wheels spin and drive and stuff

In your main drivetrain run loop (or your default driving Command's execute() for Command-Based) set each Module's speed and angle by sending your joystick objects


```java
wheel1.setSpeedAndAngle(driveJoystick, rotateJoystick); 
wheel2.setSpeedAndAngle(driveJoystick, rotateJoystick); 
wheel3.setSpeedAndAngle(driveJoystick, rotateJoystick); 
wheel4.setSpeedAndAngle(driveJoystick, rotateJoystick); 
```

<i>Or if your drive team uses Gamepads Controllers, you can initalize it as a ShakerGamepad.</i>

```java
wheel1.setSpeedAndAngle(driverGamepad); 
wheel2.setSpeedAndAngle(driverGamepad); 
wheel3.setSpeedAndAngle(driverGamepad); 
wheel4.setSpeedAndAngle(driverGamepad); 
```
</br>

And that's it! Once your PID is tuned <i>(there are PID getters and setters in TalonModule.java)</i>, you'll have a working swerve adjustable to your robot design and code layout.

