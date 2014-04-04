import java.io.File;
import java.util.ArrayList;

import lejos.nxt.Button;
import lejos.nxt.LCDOutputStream;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.TachoMotorPort;
import lejos.nxt.TouchSensor;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.addon.ColorHTSensor;
import lejos.nxt.addon.MMXRegulatedMotor;
import lejos.nxt.remote.RemoteMotor;
import lejos.robotics.Color;
import lejos.nxt.Sound;

import lejos.nxt.addon.NXTMMX;

public class Main {
	static NXTMMX multiplex = new NXTMMX(SensorPort.S2);
	static UltrasonicSensor rightSensor = new UltrasonicSensor(SensorPort.S4);
	static UltrasonicSensor frontSensor = new UltrasonicSensor(SensorPort.S3);
	static TouchSensor ts = new TouchSensor(SensorPort.S1);

	static MMXRegulatedMotor intake = new MMXRegulatedMotor(multiplex,
			NXTMMX.MMX_MOTOR_1);
	static MMXRegulatedMotor elevator = new MMXRegulatedMotor(multiplex,
			NXTMMX.MMX_MOTOR_2);

	static NXTRegulatedMotor leftWheel = Motor.B;
	static NXTRegulatedMotor rightWheel = Motor.A;
	static NXTRegulatedMotor thrower = Motor.C;

	LCDOutputStream output = new LCDOutputStream();

	static int TURN_SPEED = 200;
	static int TURN_TIME = 1500;

	static int[] shootPosn = { 52, 43 };
	static int[] scanPosn = { 90, 75 };

	public static void main(String[] args) throws InterruptedException {
		launch();
		// leftWheel.setAcceleration(leftWheel.getAcceleration()-50);
		// rightWheel.setAcceleration(rightWheel.getAcceleration()-50);
		// while(!Button.ENTER.isDown()){
		// goToPosition(shootPosn[0], shootPosn[1]);
		// Thread.sleep(250);
		// goToPosition(scanPosn[0], scanPosn[1]);
		findABall();
		fetch();
	}

	private static void stopWheels() throws InterruptedException {
		leftWheel.setSpeed(250);
		rightWheel.setSpeed(150);
		Thread.sleep(130);
		leftWheel.stop();
		rightWheel.stop();
		// rightWheel.setSpeed(130);
		// rightWheel.forward();
		// Thread.sleep(500);
		// rightWheel.stop();

	}

	private static void goBackward(int speed) {
		leftWheel.setSpeed(speed);
		rightWheel.setSpeed(speed);
		leftWheel.forward();
		rightWheel.forward();
	}

	//
	private static void goForward(int speed) {
		leftWheel.setSpeed(speed);
		rightWheel.setSpeed(speed);
		leftWheel.backward();
		rightWheel.backward();
	}

	private static void turnRight(int extraSleepTime)
			throws InterruptedException {
		leftWheel.setSpeed(250);
		rightWheel.setSpeed(250);
		leftWheel.backward();
		rightWheel.forward();
		Thread.sleep(1500 + extraSleepTime);
		leftWheel.stop();
		rightWheel.stop();
	}

	private static void turnLeft(int extraSleepTime)
			throws InterruptedException {
		leftWheel.setSpeed(250);
		rightWheel.setSpeed(250);
		leftWheel.forward();
		rightWheel.backward();
		Thread.sleep(1500 + extraSleepTime);
		leftWheel.stop();
		rightWheel.stop();
	}

	private static int getFrontSensorDistance() {
		return frontSensor.getDistance() + 10;
	}

	private static void findABall() throws InterruptedException {
		boolean foundBall = false;
		boolean foundBallBehind = false;
		turnRight(0);
		int currentDistance = rightSensor.getDistance();
		goForward(150);
		while (frontSensor.getDistance() > 10 && !foundBall && !foundBallBehind) {
			int distance = rightSensor.getDistance();
			System.out.println(distance);
			if (distance < currentDistance - 5) {
				foundBall = true;
				Sound.beep();
			} else if (distance > currentDistance + 2) {
				foundBallBehind = true;
				Sound.beep();
//			} else {
//				currentDistance = distance;
			}
		}
	}
	
	private static void fetch() throws InterruptedException {
		int distanceFromBall = rightSensor.getDistance();
		turnLeft(0);
		goBackward(150);
		intake();
		boolean thing = true;
		while (thing) {
			if (ts.isPressed()) {
				thing = false;
			}
		}
		elevator.setSpeed(500);
		elevator.backward();
		Thread.sleep(1500);
		elevator.setSpeed(0);
		elevator.backward();
	}

	private static void goToPosition(int destXCoord, int destYCoord)
			throws InterruptedException {
		int currentXCoord = rightSensor.getDistance();
		int currentYCoord = getFrontSensorDistance();
		double xDistance = destXCoord - currentXCoord;
		double yDistance = destYCoord - currentYCoord;
		boolean goRightX = xDistance < 0;
		boolean goUpY = yDistance < 0;
		int moveSpeed = 360;
		double distanceRate = 15.0;
		if (Math.abs(xDistance) < 5 && Math.abs(yDistance) < 5) {

		} else {
			if (goRightX == true) {
				turnRight(0);
				goForward(moveSpeed);
				Thread.sleep((long) (((Math.abs(xDistance) / distanceRate) * 1000)));
				stopWheels();
				turnLeft(-100);
			} else {
				turnLeft(25);
				goForward(moveSpeed);
				Thread.sleep((long) ((Math.abs(xDistance) / distanceRate) * 1000));
				stopWheels();
				turnRight(-75);
			}
			if (goUpY == true) {
				goForward(moveSpeed); // 24 degrees/cm, 360 degrees for one
										// second
				// goes 15
				// cm, 15cm/sec
				Thread.sleep((long) (((Math.abs(yDistance) / distanceRate) * 1000)));
				stopWheels();
			} else {
				goBackward(moveSpeed);
				Thread.sleep((long) ((Math.abs(yDistance) / distanceRate) * 1000));
				stopWheels();
			}
		}
	}

	private static void intake() throws InterruptedException {
		intake.setSpeed(500);
		elevator.setSpeed(500);
		intake.backward();
		elevator.backward();
		Thread.sleep(4000);
		intake.setSpeed(0);
		elevator.setSpeed(0);
		intake.backward();
		elevator.backward();
	}

	private static void lift() throws InterruptedException {
		elevator.setSpeed(500);
		thrower.setSpeed(50);
		elevator.backward();
		thrower.backward();
		Thread.sleep(4000);
		elevator.stop();
		thrower.stop();
	}

	private static void launch() throws InterruptedException {
		thrower.setSpeed(1350);
		thrower.forward();
		Thread.sleep(125);
		thrower.stop();
		Thread.sleep(1000);
		thrower.backward();
		Thread.sleep(125);
		thrower.stop();
	}

	private static void lookForBall() throws InterruptedException {
		Thread.sleep(250);
		while (!Button.ENTER.isDown()) {
			if (!ts.isPressed()) {
				goBackward(250);
				intake();
			} else {
				break;
			}
		}
		goBackward(0);
	}
}