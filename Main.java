import java.io.File;
import java.util.ArrayList;

import lejos.nxt.LCDOutputStream;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.TachoMotorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.addon.ColorHTSensor;
import lejos.nxt.addon.MMXRegulatedMotor;
import lejos.nxt.remote.RemoteMotor;
import lejos.robotics.Color;
import lejos.nxt.Sound;

import lejos.nxt.addon.NXTMMX;

public class Main {
	static NXTMMX multiplex = new NXTMMX(SensorPort.S2);
	static UltrasonicSensor Us1 = new UltrasonicSensor(SensorPort.S4);
	static UltrasonicSensor Us2 = new UltrasonicSensor(SensorPort.S3);

	static MMXRegulatedMotor intake = new MMXRegulatedMotor(multiplex,
			NXTMMX.MMX_MOTOR_1);
	static MMXRegulatedMotor elevator = new MMXRegulatedMotor(multiplex,
			NXTMMX.MMX_MOTOR_2);

	static NXTRegulatedMotor leftWheel = Motor.B;
	static NXTRegulatedMotor rightWheel = Motor.A;
	static NXTRegulatedMotor thrower = Motor.C;
	
	static int TURN_SPEED = 200;
	static int TURN_TIME = 1500;

	public static void main(String[] args) throws InterruptedException {
//		goForward(700);
		// Thread.sleep(125);
//		Thread.sleep(125);
//		rightWheel.stop();
//		leftWheel.stop();
//		thrower.stop();
//		System.out.println("stopped");
//		Thread.sleep(2700);
//		goForward(500);
//		Thread.sleep(2000);
//		goBackwards(500);
//		Thread.sleep(1000);
//		
//		turnLeft();
//		Thread.sleep(1000);
//		turnRight();
		intake();
		Thread.sleep(50);
		lift();
		Thread.sleep(100);
		launch();
	}

	private static void goForward(int speed) {
		leftWheel.setSpeed(speed);
		rightWheel.setSpeed(speed);
		leftWheel.forward();
		rightWheel.forward();
	}

	//
	private static void goBackwards(int speed) {
		leftWheel.setSpeed(speed);
		rightWheel.setSpeed(speed);
		leftWheel.backward();
		rightWheel.backward();
	}

	private static void turnLeft() throws InterruptedException {
		leftWheel.setSpeed(TURN_SPEED);
		rightWheel.setSpeed(TURN_SPEED);
		leftWheel.backward();
		rightWheel.forward();
		Thread.sleep(TURN_TIME);
	}

	private static void turnRight() throws InterruptedException {
		leftWheel.setSpeed(TURN_SPEED);
		rightWheel.setSpeed(TURN_SPEED);
		leftWheel.forward();
		rightWheel.backward();
		Thread.sleep(1750);
		}
	
	private static void intake() throws InterruptedException {
		intake.setSpeed(500);
		elevator.setSpeed(500);
		intake.backward();
		elevator.backward();
		Thread.sleep(2500);
		intake.setSpeed(0);
		elevator.setSpeed(0);
		intake.backward();
		elevator.backward();
//		intake.stop();
//		elevator.stop();
	}
	
	private static void lift() throws InterruptedException {
		elevator.setSpeed(500);
		thrower.setSpeed(50);
		elevator.backward();
		thrower.backward();
		Thread.sleep(3500);
		elevator.stop();
		thrower.stop();
	}
	
	private static void launch() throws InterruptedException {
		thrower.setSpeed(700);
		thrower.forward();
		Thread.sleep(125);
		thrower.stop();
		Thread.sleep(1000);
		thrower.backward();
		Thread.sleep(125);
		thrower.stop();
	}
}