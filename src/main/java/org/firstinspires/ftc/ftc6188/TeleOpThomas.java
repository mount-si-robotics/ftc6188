/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.ftc6188;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOP", group="Iterative Opmode")  // @AutonomousBlue(...) is the other common choice
public class TeleOpThomas extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorRightFront;
    private DcMotor motorRightBack;
    private DcMotor motorLeftBack;
    private DcMotor motorLeftFront;
    private Servo buttonPusher;
    private ColorSensor modernRobotics;
    private OpticalDistanceSensor OpticalDistance;
    private GyroSensor MrGyro;

    private float buttonPusherPosition = 0;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        motorRightFront = hardwareMap.dcMotor.get("RFMotor");
        motorRightBack = hardwareMap.dcMotor.get("RBMotor");
        motorLeftFront = hardwareMap.dcMotor.get("LFMotor");
        motorLeftBack = hardwareMap.dcMotor.get("LBMotor");
        buttonPusher = hardwareMap.servo.get("ButtonPusherCRServo");
        modernRobotics = hardwareMap.colorSensor.get("MRCSensor");
        OpticalDistance = hardwareMap.opticalDistanceSensor.get("ODSensor");
        MrGyro = hardwareMap.gyroSensor.get("GSensor");

        buttonPusher.setPosition(0);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        float left;
        float right;

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
        if(gamepad1.right_trigger > 0.25)
        {
            buttonPusherPosition+=.004f;
        }
        if(gamepad1.right_bumper)
        {
            buttonPusherPosition-=.004f;
        }
        Range.clip(buttonPusherPosition,0,1);
        telemetry.addData("buttonPusherPosition ",buttonPusherPosition);
        buttonPusher.setPosition(buttonPusherPosition);

        telemetry.update();

        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;

        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);

        if(gamepad1.left_trigger > 0.25)
        {
            right/=4;
            left/=4;
        }
        motorRightFront.setPower(right);
        motorLeftFront.setPower(left);
        motorRightBack.setPower(right);
        motorLeftBack.setPower(left);
    }
    @Override
    public void stop() {
    }
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        }
        if (index > 16) {
            index = 16;
        }
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }
        return dScale;
    }
}
