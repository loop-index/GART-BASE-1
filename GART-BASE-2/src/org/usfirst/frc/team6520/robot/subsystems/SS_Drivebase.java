package org.usfirst.frc.team6520.robot.subsystems;

import org.usfirst.frc.team6520.robot.Robot;
import org.usfirst.frc.team6520.robot.RobotMap;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class SS_Drivebase extends Subsystem {

    public Spark left = new Spark(0);
    public Spark right = new Spark(1);
//    public Spark pg = new Spark(2);
    
    public Encoder leftEncoder = new CIMCoder(0, 1, false, EncodingType.k4X);
    public Encoder rightEncoder = new CIMCoder(2, 3, true, EncodingType.k4X);
//	public Encoder enc = new Encoder(4,5, true, EncodingType.k4X);
    
    public AHRS navX = new AHRS(SPI.Port.kMXP);
    
//    public Pixy pixy = new Pixy(9, 0);
    public PixyCam pixy = new PixyCam();
    
//    public int PIXY_I2C = 0x54;
//    public PixyI2C pixy = new PixyI2C(0, argPixy, argPixyPacket, argPixyException, argValues)
    
    public int PPR = 20;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
//    
//    public void leftDist(double centimeters){
    
    public void driveDist(double centimeters){
    	while (leftEncoder.getRaw() < 360){
//    		leftEncoder.
    	}
    }
    
    public void driveTimer(double time){
    	leftEncoder.reset();
    	rightEncoder.reset();
    	double start = Timer.getFPGATimestamp();
    	double acceptableDifference = 5;
    	while (Timer.getFPGATimestamp() - start < time){
    		SmartDashboard.putNumber("time", Timer.getFPGATimestamp() - start);
    		if (leftEncoder.getRaw()/PPR - rightEncoder.getRaw()/PPR > acceptableDifference){
        		left.set(-0.7);
        		right.set(0.5);
    		} 
    		else if (rightEncoder.getRaw()/PPR - leftEncoder.getRaw()/PPR > acceptableDifference){
        		left.set(-0.5);
        		right.set(0.7);
    		}
    		else {
        		left.set(-0.5);
        		right.set(0.5);
    		}
    		RobotMap.update();
    	}
    	left.stopMotor();
    	right.stopMotor();
    }
    
    public void turn(int angle){
    	navX.reset();
    	leftEncoder.reset();
    	rightEncoder.reset();
    	double turnDiameter = 58;
    	double required = turnDiameter * Math.PI * (angle/360.0);
    	SmartDashboard.putNumber("required dist", required);
    	while (navX.getAngle() < angle){
    		RobotMap.update();
    		left.set(0.25);
    		right.set(0.25);
    	}
    	left.stopMotor();
    	right.stopMotor();
    	
    }
    
    public void turnPID(double angle){
    	navX.reset();
    	double heading = navX.getAngle();
    	double speed = 0.6;
    	double kP = 0.08;
    	double E = 0;
    	while (navX.getAngle() - heading < angle){
    		SmartDashboard.putNumber("E", navX.getAngle() - heading);
    		RobotMap.update();
    		E = angle - navX.getAngle();
    		left.set(E/angle * speed * kP);
    		right.set(E/angle * speed * kP);
    	}
    	left.stopMotor();
    	right.stopMotor();
    }
    
    public AnalogInput analog = new AnalogInput(0);
    public DigitalInput digital = new DigitalInput(9);
    
    public void pixy(){
    	SmartDashboard.putNumber("pixy", pixy.getTargetX());
//    	pixy.readPixyPacket();
    }
    
    public int accErr = 5;
    public void trackX(){
    	double X = pixy.getTargetX();
    	if (X < 160 - accErr){
    		right.set(0.2);
    		left.set(0.2);
    	}
    	else if (X > 160 + accErr){
    		left.set(-0.2);
    		right.set(-0.2);
    	} else {
    		left.stopMotor();
    		right.stopMotor();
    	}
    }
    
//    public void turnEnc(int angle){
//    	enc.reset();
//    	int value = angle + 1;
//    	while (value != angle){
//    		value = (int) (enc.get()/500.0 * 360 % 360);
//    		SmartDashboard.putNumber("enc", value);
//    		pg.set(-0.6);
//    	}
//    	pg.stopMotor();
//    }
    
    public void followX (int diff){
    	if (Robot.vis.centerX < 160 - diff){
        	left.set(-0.1);
        	right.set(-0.1);
    	} else if (Robot.vis.centerX > 160 + diff){
        	left.set(0.1);
        	right.set(0.1);
    	} else {
    		left.stopMotor();
    		right.stopMotor();
    	}
    }
}


