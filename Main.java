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
	static int TURN_RIGHT = -200;
	static int TURN_RIGHT_UP = -75;
	static int TURN_LEFT = 25;
	static int TURN_LEFT_UP = -75;

	static int[] shootPosn = { 52, 43 };
	static int[] scanPosn = { 90, 75 };

	public static void main(String[] args) throws InterruptedException {
		Thread.sleep(250);
		goToPosition(rightSensor.getDistance(), shootPosn[1]);
		elevator.setSpeed(500);
		elevator.backward();
		Thread.sleep(2000);
		elevator.stop();
		launch();
		Thread.sleep(200);
		while (!Button.ENTER.isDown()) {
			goToPosition(scanPosn[0], scanPosn[1]);
			boolean successful = findABall();
			if (successful) {
				fetch();
				goToHoop();
			} else {
				turnLeft(TURN_LEFT_UP);
			}
		}
		 while (!Button.ENTER.isDown()) {
		 turnRight(0);
		 }
		
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

	private static void straightenOut() throws InterruptedException {
		long time = (long) (((getFrontSensorDistance() / 7.5) * 1000));
		goForward(180);
		Thread.sleep(time);
		goForward(0);
		Thread.sleep(50);
		goBackward(180);
		Thread.sleep(time);
		goBackward(0);
	}

	private static boolean findABall() throws InterruptedException {
		boolean foundBall = false;
		boolean foundBallBehind = false;
		turnRight(TURN_RIGHT-50); //Added more weight (wait?) to the global variable
		int currentDistance = rightSensor.getDistance();
		goForward(150);
		while (frontSensor.getDistance() > 10 && !foundBall && !foundBallBehind) {
			int distance = rightSensor.getDistance();
			System.out.println(distance);
			if (distance < currentDistance - 5) {
				foundBall = true;
				Sound.beep();
			} else if (distance > currentDistance + 2) {
				foundBall = true;
				Sound.beep();
				// } else {
				// currentDistance = distance;
			}
		}
		return foundBall;
	}

	private static void fetch() throws InterruptedException {
		turnLeft(TURN_LEFT_UP);
		int currentDistance = frontSensor.getDistance();
		int totalDistance = 0;
		goBackward(250);
		intake.setSpeed(500);
		elevator.setSpeed(500);
		intake.backward();
		elevator.backward();
		boolean thing = true;
		while (thing) {
			if (ts.isPressed()) {
				thing = false;
				Sound.beep();
			}
		}
		stopWheels();
		intake.setSpeed(0);
		elevator.setSpeed(0);
		intake.backward();
		elevator.backward();
		intake.setSpeed(500);
		elevator.setSpeed(500);
		intake.backward();
		elevator.backward();
		Thread.sleep(2000);
		intake.setSpeed(0);
		elevator.setSpeed(0);
		intake.backward();
		elevator.backward();
	}

	private static void goToHoop() throws InterruptedException {
		goForward(250);
		Thread.sleep(2000);
		stopWheels();
		goToPosition(shootPosn[0], shootPosn[1]);
		liftAndLaunch();
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

		} if (Math.abs(xDistance) > 5) {
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
	
	@SuppressWarnings("deprecation")
	private static void liftAndLaunch() throws InterruptedException {
		elevator.setSpeed(500);
		thrower.setSpeed(20);
		elevator.backward();
		thrower.backward();
		Thread.sleep(250);
		thrower.stop();
		//thrower.lock(10);
		Thread.sleep(3750);
		Sound.beep();
		elevator.setSpeed(0);
		// thrower.flt();
		//thrower.stop();
		Sound.beep();
		//thrower.forward();
		// Thread.sleep(1);
		//thrower.stop();
		thrower.setSpeed(1350);
		thrower.forward();
		Thread.sleep(125);
		thrower.stop();
		Thread.sleep(1000);
		thrower.backward();
		Thread.sleep(125);
		thrower.stop();

	}

	// private static void launch() throws InterruptedException {
	// thrower.setSpeed(1350);
	// thrower.forward();
	// Thread.sleep(125);
	// thrower.stop();
	// Thread.sleep(1000);
	// thrower.backward();
	// Thread.sleep(125);
	// // thrower.stop();
	// thrower.setSpeed(0);
	// thrower.forward();
	// }

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