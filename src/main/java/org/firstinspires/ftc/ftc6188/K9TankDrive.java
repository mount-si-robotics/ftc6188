/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.ftc6188;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

import java.nio.channels.DatagramChannel;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class K9TankDrive extends OpMode {

	/*
	 * Note: the configuration of the servos is such that
	 * as the arm servo approaches 0, the arm position moves up (away from the floor).
	 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
	 */
    // TETRIX VALUES.
    final static double ARM_MIN_RANGE  = 0.00;
    final static double ARM_MAX_RANGE  = 0.50;
    final static double CLAW_MIN_RANGE  = 0.50;
    final static double CLAW_MAX_RANGE  = 1.00;

	// position of the arm servo.
	double armPosition;

	// amount to change the arm servo position.
	double armDelta = 0.05;

	// position of the claw servo
	double clawPosition;

	// amount to change the claw servo position by
	double clawDelta = 0.05;

	double throwPosition;
	double ThrowPosition2;

	double slidePosition;

	double throwDelra = .1;

	//declaring sensors
	private ColorSensor modernRobotics;
	private GyroSensor gyroTest;
	private UltrasonicSensor ultra;

	private LegacyModule legacy;
	//delcaring motors
	DcMotor motorRightFront;
	DcMotor motorRightBack;
	DcMotor motorLeftBack;
	DcMotor motorLeftFront;
	DcMotor arm;
	//declaring servos
	Servo cServo;
	Servo cServo2;
	Servo lever1;
	Servo lever2;
	Servo slide;


	/**
	 * Constructor
	 */
	public K9TankDrive() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {

		//give motors name in config
		motorRightFront = hardwareMap.dcMotor.get("motor_rf");
		motorRightBack = hardwareMap.dcMotor.get("motor_rb");
		motorLeftFront = hardwareMap.dcMotor.get("motor_lf");
		motorLeftBack = hardwareMap.dcMotor.get("motor_lb");

		arm = hardwareMap.dcMotor.get("arm");

		//set direction for motors to spin
		motorLeftBack.setDirection(DcMotor.Direction.FORWARD);
		motorLeftFront.setDirection(DcMotor.Direction.FORWARD);
		motorRightBack.setDirection(DcMotor.Direction.REVERSE);
		motorRightFront.setDirection(DcMotor.Direction.REVERSE);

		//give name to servos in config
		cServo = hardwareMap.servo.get("cServo");
		cServo2 = hardwareMap.servo.get("cServo2");
		lever1 = hardwareMap.servo.get("lever1");
		lever2 = hardwareMap.servo.get("lever2");
		slide = hardwareMap.servo.get("slide");

		//give the sensors a name in the config
		modernRobotics = hardwareMap.colorSensor.get("mr");
		gyroTest = hardwareMap.gyroSensor.get("gyro");
		ultra = hardwareMap.ultrasonicSensor.get("ultra");
		legacy = hardwareMap.legacyModule.get("legacy");

		// assign base positions for servos
		armPosition = 0.0;
		clawPosition = 1.0;
		throwPosition = 1.0;
		ThrowPosition2 = 0.0;
		slidePosition = slide.MAX_POSITION;


	}

	/*
	 * This method will be called repeatedly in a loop
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		/*
		 * Gamepad 1
		 * 
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */

        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.
		float left;
		float right;
		float armSpeed;
		float slideSpeed;


			 left = -gamepad1.left_stick_y;
			 right = -gamepad1.right_stick_y;
			 armSpeed = - gamepad2.left_stick_y / 1.7f;
			 slideSpeed = gamepad2.right_stick_y;

		// clip the right/left values so that the values never exceed +/- 1
		right = Range.clip(right, -1, 1);
		left = Range.clip(left, -1, 1);
		armSpeed = Range.clip(armSpeed, -1, 1);
		slideSpeed = Range.clip(slideSpeed, -.002f, .002f);

		// scale the joystick value to make it easier to control
		// the robot more precisely at slower speeds.
		right = (float)scaleInput(right);
		left =  (float)scaleInput(left);

		//change the speed of the clawArm as it is extended
		float additive_speed = 0.0f;
			if(1 - slide.getPosition() > .25 && 1-slide.getPosition() < .5)
				additive_speed = .03f;
			else if(1 - slide.getPosition() > .5 && 1- slide.getPosition() < .8)
				additive_speed = .07f;
			else if(1 - slide.getPosition() > .8)
				additive_speed = .13f;
			else
			additive_speed = 0.0f;

		if(armSpeed < 0)
			additive_speed *= -1;
			if(armSpeed != 0)
				armSpeed = (float) scaleInput(armSpeed) + additive_speed;
			else
				armSpeed = (float) scaleInput(armSpeed);

		// write the values to the motors
		if(motorRightFront.getDirection() != DcMotor.Direction.REVERSE) {
			motorRightFront.setPower(right);
			motorLeftFront.setPower(left);
			motorRightBack.setPower(right);
			motorLeftBack.setPower(left);
		}
		else
		{
			motorRightFront.setPower(left);
			motorLeftFront.setPower(right);
			motorRightBack.setPower(left);
			motorLeftBack.setPower(right);
		}
		//setting the spead of linearSlide
		arm.setPower(armSpeed);
		slidePosition = slidePosition + slideSpeed;

		slidePosition = Range.clip(slidePosition, .05, slide.MAX_POSITION);
		slide.setPosition(slidePosition);



		//allows user to drive backwards by pressing a and forward for b for driver 1
		if(gamepad1.a)
		{

				motorRightBack.setDirection(DcMotor.Direction.FORWARD);
				motorRightFront.setDirection(DcMotor.Direction.FORWARD);
				motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
				motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
		}
		else if(gamepad1.b)
		{
			motorRightBack.setDirection(DcMotor.Direction.REVERSE);
			motorRightFront.setDirection(DcMotor.Direction.REVERSE);
			motorLeftBack.setDirection(DcMotor.Direction.FORWARD);
			motorLeftFront.setDirection(DcMotor.Direction.FORWARD);

		}

		//setting the position of the lever arms for driver 2
		if(gamepad2.left_trigger > 0.25)
		{
			armPosition += armDelta;
		}

        if (gamepad2.left_bumper) {
            armPosition -= armDelta;
        }

		if (gamepad2.right_trigger > 0.25) {

			clawPosition -=clawDelta;
		}

		if(gamepad2.right_bumper)
		{
			clawPosition +=clawDelta;
		}

		//opening the claw on a and closing it on b for driver 2
		if(gamepad2.a)
		{
			throwPosition +=throwDelra;
			ThrowPosition2-=throwDelra;
		}
		if(gamepad2.b)
		{
			throwPosition -= throwDelra;
			ThrowPosition2 += throwDelra;
		}



		// clip the position values so that they never exceed their allowed range.
		armPosition = Range.clip(armPosition, ARM_MIN_RANGE, ARM_MAX_RANGE);
		lever1.setPosition(armPosition);
		clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
		lever2.setPosition(clawPosition);
		throwPosition = Range.clip(throwPosition,.1f,1);
		cServo.setPosition(throwPosition);
		ThrowPosition2 = Range.clip(ThrowPosition2,.0,.7f);
		cServo2.setPosition(ThrowPosition2);
		// write position values to the wrist and claw servo





		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
		telemetry.addData("addditiveSpeed", additive_speed);
		telemetry.addData("armSpeed",armSpeed);
		telemetry.addData("speedSlideServo",slideSpeed);
		telemetry.addData("slide servo position",slide.getPosition());
		telemetry.addData("Direction", motorRightBack.getDirection());
		telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("cServo",cServo.getPosition());
        telemetry.addData("cServo 2", cServo2.getPosition());
		telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
		telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 * 
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}
	
	/*
	 * This method scales the joystick input so for low joystick values, the 
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };
		
		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);
		
		// index should be positive.
		if (index < 0) {
			index = -index;
		}

		// index cannot exceed size of array minus 1.
		if (index > 16) {
			index = 16;
		}

		// get value from the array.
		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		// return scaled value.
		return dScale;
	}

}
