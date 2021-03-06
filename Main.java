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
	static int TURN_RIGHT = 0;
	static int TURN_RIGHT_UP = 25;
	static int TURN_LEFT = 0;
	static int TURN_LEFT_UP = 50;
	static int ARENA_LENGTH = 106;

	static int[] shootPosn = { 52, 38 }; // 40
	static int[] scanPosn = { 90, 75 };
	static boolean justShot = false;

	public static void main(String[] args) throws InterruptedException {
		Thread.sleep(250);
		goToPosition(rightSensor.getDistance(), shootPosn[1], true);
		elevator.setSpeed(500);
		elevator.backward();
		Thread.sleep(2000);
		elevator.stop();
		launch();
		justShot = true;
		Thread.sleep(400);
		while (!Button.ENTER.isDown()) {
			goToPosition(scanPosn[0], scanPosn[1], false);
			boolean successful = findABall();
			if (successful) {
				if (fetch()) {
					goToHoop();
					// straightenOut();
					justShot = true;
				} else {
					goForward(350);
					Thread.sleep(1500);
				}
			} else {
				goBackward(500);
				Thread.sleep(500);
				turnLeft(TURN_LEFT_UP);
			}
		}
		// while (!Button.ENTER.isDown()) {
		// turnRight(0);
		// }

	}

	private static void stopWheels() throws InterruptedException {
		leftWheel.setSpeed(250);
		rightWheel.setSpeed(150);
		Thread.sleep(130);
		leftWheel.stop();
		rightWheel.stop();
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
		leftWheel.setSpeed(200);
		rightWheel.setSpeed(200);
		leftWheel.backward();
		rightWheel.forward();
		Thread.sleep((long) (1800 + (extraSleepTime * 1.2)));
		leftWheel.stop();
		rightWheel.stop();
	}

	private static void turnLeft(int extraSleepTime)
			throws InterruptedException {
		leftWheel.setSpeed(200);
		rightWheel.setSpeed(200);
		leftWheel.forward();
		rightWheel.backward();
		Thread.sleep((long) (1800 + (extraSleepTime * 1.2)));
		leftWheel.stop();
		rightWheel.stop();
	}

	private static int getFrontSensorDistance() {
		return frontSensor.getDistance() + 10;
	}

	private static void straightenOut() throws InterruptedException {
		long time = (long) (((getFrontSensorDistance() / 15.0) * 1000));
		goForward(315);
		Thread.sleep(time - 600);
		stopWheels();
		leftWheel.flt();
		rightWheel.flt();
		Thread.sleep(50);
		goBackward(200);
		Thread.sleep(time);
		goBackward(0);
		Thread.sleep(250);
	}

	private static boolean findABall() throws InterruptedException {
		boolean foundBall = false;
		boolean foundBallBehind = false;
		turnRight(TURN_RIGHT - 50); // Added more weight (wait?) to the global
									// variable
		int currentDistance = rightSensor.getDistance();
		goForward(200);
		while (frontSensor.getDistance() > 10 && !foundBall) {
			int distance = rightSensor.getDistance();
			System.out.println(distance);
			if (distance < currentDistance - 4) {
				foundBall = true;
				Thread.sleep(100);
				Sound.beep();
			} else if (distance > currentDistance + 4) {
				if (!foundBallBehind) {
					currentDistance = distance;
					foundBallBehind = true;
				}
				// foundBall = true;
				// goBackward(200);
				// Thread.sleep(500);
				// stopWheels();
				// Sound.beep();
				// } else {
				// currentDistance = distance;
			}
		}
		return foundBall;
	}

	private static boolean fetch() throws InterruptedException {
		turnLeft(TURN_LEFT_UP);
		int currentDistance = getFrontSensorDistance();
		goBackward(200);
		intake.setSpeed(500);
		elevator.setSpeed(500);
		intake.backward();
		elevator.backward();
		boolean thing = true;

		while (thing && (ARENA_LENGTH - getFrontSensorDistance() > 0)) {
			if (ts.isPressed()) {
				thing = false;
				Sound.beep();
			}
		}
		stopWheels();
		intake.setSpeed(0);
		elevator.setSpeed(0);
		if (!thing) {

			intake.backward();
			elevator.backward();
			intake.setSpeed(500);
			elevator.setSpeed(500);
			intake.backward();
			elevator.backward();
			Thread.sleep(3000);
			intake.setSpeed(0);
			elevator.setSpeed(0);
			intake.backward();
			elevator.backward();
			// back into wall to correct
			Sound.beep();

			goBackward(210);
			Thread.sleep(180);
			Sound.beep();
			stopWheels();
			return true;
		} else {
			return false;
		}

	}

	private static void goToHoop() throws InterruptedException {
		goForward(350);
		Thread.sleep(2000);
		stopWheels();
		goToPosition(shootPosn[0], shootPosn[1], true);
		liftAndLaunch();
	}

	private static void goToPosition(int destXCoord, int destYCoord,
			boolean goingToHoop) throws InterruptedException {
		int currentXCoord = rightSensor.getDistance();
		if ((currentXCoord > 100 || currentXCoord < 15) && !goingToHoop) {
			Sound.beep(); // when the ball rolls infront of the hoop
			currentXCoord = 50;
		}
		int currentYCoord = getFrontSensorDistance();
		if ((currentYCoord > 100 || currentYCoord < 15) && !goingToHoop) {
			Sound.beep();
			currentYCoord = 40;
		}
		double xDistance = destXCoord - currentXCoord;
		double yDistance = destYCoord - currentYCoord;
		boolean goRightX = xDistance < 0;
		boolean goUpY = yDistance < 0;
		int moveSpeed = 720; //
		double distanceRate = 28; // 15.0
		if (Math.abs(xDistance) < 5 && Math.abs(yDistance) < 5) {

		}
		if (Math.abs(xDistance) > 5) {
			if (goRightX == true) {
				turnRight(TURN_RIGHT);
				goForward(moveSpeed);
				Thread.sleep((long) (((Math.abs(xDistance) / distanceRate) * 1000)));
				stopWheels();
				turnLeft(TURN_LEFT_UP);
			} else {
				turnLeft(TURN_LEFT);
				goForward(moveSpeed);
				Thread.sleep((long) ((Math.abs(xDistance) / distanceRate) * 1000));
				stopWheels();
				if (!goingToHoop && justShot == true) {
					straightenOut();
					justShot = false;
				}
				turnRight(TURN_RIGHT_UP);
			}
		}
		if (Math.abs(yDistance) > 5) {
			if (goUpY == true) {
				goForward(moveSpeed); // 24 degrees/cm, 360 degrees for one
										// second
				// goes 15
				// cm, 15cm/sec
				Thread.sleep((long) (((Math.abs(yDistance) / distanceRate) * 1000) - 500));
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

	@SuppressWarnings("deprecation")
	private static void lift() throws InterruptedException {
		elevator.setSpeed(500);
		thrower.setSpeed(20);
		elevator.backward();
		thrower.backward();
		Thread.sleep(2000);
		thrower.lock(10);
		Thread.sleep(2500);
		Sound.beep();
		elevator.stop();
		// thrower.flt();
		thrower.stop();
		Sound.beep();
		thrower.forward();
		// Thread.sleep(1);
		thrower.stop();

	}

	private static void liftAndLaunch() throws InterruptedException {
		elevator.setSpeed(500);
		thrower.setSpeed(20);
		elevator.backward();
		thrower.backward();
		Thread.sleep(250);
		thrower.stop();
		// thrower.lock(10);
		Thread.sleep(3750);
		Sound.beep();
		elevator.setSpeed(0);
		// thrower.flt();
		// thrower.stop();
		Sound.beep();
		// thrower.forward();
		// Thread.sleep(1);
		// thrower.stop();
		thrower.setSpeed(1350);
		thrower.forward();
		Thread.sleep(125);
		thrower.stop();
		Thread.sleep(1000);
		thrower.backward();
		Thread.sleep(125);
		thrower.stop();
		Thread.sleep(400);

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
}