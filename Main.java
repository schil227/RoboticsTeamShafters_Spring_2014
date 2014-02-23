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
	 static NXTMMX multiplex = new NXTMMX(SensorPort.S4);
	 static UltrasonicSensor Us1 = new UltrasonicSensor(SensorPort.S2);
	 static UltrasonicSensor Us2 = new UltrasonicSensor(SensorPort.S3);

	 static MMXRegulatedMotor m1 = new MMXRegulatedMotor(multiplex,
	 NXTMMX.MMX_MOTOR_1);
	 static MMXRegulatedMotor m2 = new MMXRegulatedMotor(multiplex,
	 NXTMMX.MMX_MOTOR_2);
	//
	static NXTRegulatedMotor leftWheel = Motor.B;
	static NXTRegulatedMotor rightWheel = Motor.C;
	static NXTRegulatedMotor thresher = Motor.A;

	public static void main(String[] args) throws InterruptedException {
		 goForward(700);
//		 Thread.sleep(125);
		 Thread.sleep(125);
		 rightWheel.stop();
		 leftWheel.stop();
		 thresher.stop();
		System.out.println("stopped");
		Thread.sleep(2700);
	}

	 private static void goForward(int speed) {
	 leftWheel.setSpeed(speed);
	 rightWheel.setSpeed(speed);
	 leftWheel.forward();
	 rightWheel.forward();
	 }
	//
	// private static void goBackwards(int speed) {
	// leftWheel.setSpeed(speed);
	// rightWheel.setSpeed(speed);
	// leftWheel.backward();
	// rightWheel.backward();
	// }
	//
	// private static void findBall() {
	//
	// }
	//
	// private static void shootBall() {
	//
	// }
}
