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

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushuBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name = "AutonomousBlue")
public class AutonomousBlue extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    public final float CIRCUMFENCE = (float)(4.00 * Math.PI);
    public final int ENCODERTICKS = 1120;
    public final float GEARRATIO = .5f;

    private DcMotor motorLeftFront;
    private DcMotor motorLeftBack;
    private DcMotor motorRightFront;
    private DcMotor motorRightBack;
    private DcMotor ballLauncher;
    private Servo buttonPusher;
    private ColorSensor modernRobotics;
    private OpticalDistanceSensor OpticalDistance;
    private GyroSensor sensorType;
    private ModernRoboticsI2cGyro MrGyro;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorRightFront = hardwareMap.dcMotor.get("RFMotor");
        motorRightBack = hardwareMap.dcMotor.get("RBMotor");
        motorLeftFront = hardwareMap.dcMotor.get("LFMotor");
        motorLeftBack = hardwareMap.dcMotor.get("LBMotor");

        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);

        buttonPusher = hardwareMap.servo.get("ButtonPusherCRServo");

        modernRobotics = hardwareMap.colorSensor.get("MRCSensor");
        OpticalDistance = hardwareMap.opticalDistanceSensor.get("ODSensor");
        sensorType = hardwareMap.gyroSensor.get("GSensor");

        MrGyro = (ModernRoboticsI2cGyro) sensorType;

        buttonPusher.setPosition(0);

        resetEncoders();
        checkEncoder();
        runEncoders();

        MrGyro.calibrate();
        while(MrGyro.isCalibrating())
        {
        }
        waitForStart();
        runtime.reset();

        moveRobot2(52,.2f);
        turnUsingRightMotors(70,.2f,0);
        moveRobot2(-9,.2f);
        turnUsingRightMotors(45,.2f,0);
        CheckBeaconForBlue(.1f,5);
        moveRobot2(-14,.2f);
        buttonPusher.setPosition(.3f);
        setMotorSpeed(-.1f);
        sleep(4000);
    }
    public void moveRobot(double distance, float speed) {
        double ticksToInches = (ENCODERTICKS * GEARRATIO) / CIRCUMFENCE;
        int PositionTarget1 = motorLeftBack.getCurrentPosition() + (int) (distance * ticksToInches);
        int PositionTarget2 = motorRightFront.getCurrentPosition() + (int) (distance * ticksToInches);
        int PositionTarget3 = motorRightBack.getCurrentPosition() + (int) (distance * ticksToInches);
        int PositionTarget4 = motorLeftFront.getCurrentPosition() + (int) (distance * ticksToInches);

        motorLeftBack.setTargetPosition(PositionTarget1);
        motorRightFront.setTargetPosition(PositionTarget2);
        motorRightBack.setTargetPosition(PositionTarget3);
        motorLeftFront.setTargetPosition(PositionTarget4);

        SetEncoderPositionToRun();
        setMotorSpeed(speed);
        while (motorLeftFront.isBusy()) {
            telemetry.addData("angle",
                    MrGyro.getHeading());

            telemetry.update();
        }
        setMotorSpeed(0);
        runEncoders();
    }
    public void moveRobot2(double distance, float speed) {
        sleep(100);
        int targetAngle = -MrGyro.getIntegratedZValue();
        double startTime = runtime.time();

        int headingerror;
        int currentheading;
        float driveConstant= .003f;
        float midPower = speed;
        float drivesteering;
        float leftPower,rightPower;
        double ticksToInches = (ENCODERTICKS * GEARRATIO) / CIRCUMFENCE;
        int PositionTarget1 = motorLeftBack.getCurrentPosition() + (int) (distance * ticksToInches);
        int PositionTarget2 = motorRightFront.getCurrentPosition() + (int) (distance * ticksToInches);
        int PositionTarget3 = motorRightBack.getCurrentPosition() + (int) (distance * ticksToInches);
        int PositionTarget4 = motorLeftFront.getCurrentPosition() + (int) (distance * ticksToInches);

        motorLeftBack.setTargetPosition(PositionTarget1);
        motorRightFront.setTargetPosition(PositionTarget2);
        motorRightBack.setTargetPosition(PositionTarget3);
        motorLeftFront.setTargetPosition(PositionTarget4);

        SetEncoderPositionToRun();
        setMotorSpeed(speed);

        while (motorLeftFront.isBusy() &&  runtime.time() < startTime+6) {

            currentheading = -MrGyro.getIntegratedZValue();
            headingerror = targetAngle - currentheading;

            drivesteering = headingerror * driveConstant;
            if(distance < 0)
                drivesteering *=-1;
            leftPower = midPower + drivesteering;
            if(leftPower > 1)
                leftPower = 1;
            if(leftPower < 0)
                leftPower = 0;
            rightPower = midPower - drivesteering;
            if(rightPower > 1)
                rightPower = 1;
            if(rightPower < 0)
                rightPower = 0;
            motorLeftBack.setPower(leftPower);
            motorLeftFront.setPower(leftPower);
            motorRightBack.setPower(rightPower);
            motorRightFront.setPower(rightPower);
            telemetry.addData("leftPower",
                    leftPower);
            telemetry.addData("rightPower",
                    rightPower);
            telemetry.addData("drivestering",
                    drivesteering);
            telemetry.addData("angle",
                    currentheading);
            telemetry.update();
        }
        setMotorSpeed(0);
        runEncoders();
    }
    public void moveRobot2(double distance, float speed, int targetAngle) {
        sleep(100);
        double startTime = runtime.time();

        int headingerror;
        int currentheading;
        float driveConstant= .003f;
        float midPower = speed;
        float drivesteering;
        float leftPower,rightPower;
        double ticksToInches = (ENCODERTICKS * GEARRATIO) / CIRCUMFENCE;
        int PositionTarget1 = motorLeftBack.getCurrentPosition() + (int) (distance * ticksToInches);
        int PositionTarget2 = motorRightFront.getCurrentPosition() + (int) (distance * ticksToInches);
        int PositionTarget3 = motorRightBack.getCurrentPosition() + (int) (distance * ticksToInches);
        int PositionTarget4 = motorLeftFront.getCurrentPosition() + (int) (distance * ticksToInches);

        motorLeftBack.setTargetPosition(PositionTarget1);
        motorRightFront.setTargetPosition(PositionTarget2);
        motorRightBack.setTargetPosition(PositionTarget3);
        motorLeftFront.setTargetPosition(PositionTarget4);
        SetEncoderPositionToRun();
        setMotorSpeed(speed);

        while (motorLeftFront.isBusy() &&  runtime.time() < startTime+6) {

            currentheading = -MrGyro.getIntegratedZValue();

            headingerror = targetAngle - currentheading;
            drivesteering = headingerror * driveConstant;
            leftPower = midPower + drivesteering;
            if(leftPower > 1)
                leftPower = 1;
            if(leftPower < 0)
                leftPower = 0;
            rightPower = midPower - drivesteering;
            if(rightPower > 1)
                rightPower = 1;
            if(rightPower < 0)
                rightPower = 0;
            motorLeftBack.setPower(leftPower);
            motorLeftFront.setPower(leftPower);
            motorRightBack.setPower(rightPower);
            motorRightFront.setPower(rightPower);
            telemetry.addData("leftPower",
                    leftPower);
            telemetry.addData("rightPower",
                    rightPower);
            telemetry.addData("drivestering",
                    drivesteering);
            telemetry.addData("angle",
                    currentheading);
            telemetry.update();
        }
        setMotorSpeed(0);
        runEncoders();
    }
    public boolean MotorsBusy()
    {
        return motorLeftBack.isBusy() && motorLeftFront.isBusy() && motorRightBack.isBusy() && motorRightFront.isBusy();
    }
    public void resetEncoders()
    {
        motorLeftFront.setMode
                (DcMotor.RunMode.STOP_AND_RESET_ENCODER
                );
        motorRightFront.setMode
                (DcMotor.RunMode.STOP_AND_RESET_ENCODER
                );
        motorLeftBack.setMode
                (DcMotor.RunMode.STOP_AND_RESET_ENCODER
                );
        motorRightBack.setMode
                (DcMotor.RunMode.STOP_AND_RESET_ENCODER
                );
    }
    public void stopEncoders()
    {
        motorLeftFront.setMode
                (DcMotor.RunMode.RUN_WITHOUT_ENCODER
                );
        motorRightFront.setMode
                (DcMotor.RunMode.RUN_WITHOUT_ENCODER
                );
        motorLeftBack.setMode
                (DcMotor.RunMode.RUN_WITHOUT_ENCODER
                );
        motorRightBack.setMode
                (DcMotor.RunMode.RUN_WITHOUT_ENCODER
                );
    }
    public void runEncoders()
    {
        motorLeftFront.setMode
                (DcMotor.RunMode.RUN_USING_ENCODER
                );
        motorRightFront.setMode
                (DcMotor.RunMode.RUN_USING_ENCODER
                );
        motorLeftBack.setMode
                (DcMotor.RunMode.RUN_USING_ENCODER
                );
        motorRightBack.setMode
                (DcMotor.RunMode.RUN_USING_ENCODER
                );
    }

    public void setMotorSpeed(float speed)
    {

        motorRightFront.setPower(speed);
        motorLeftFront.setPower(speed);
        motorLeftBack.setPower(speed);
        motorRightBack.setPower(speed);
    }

    public void checkEncoder()
    {
        while(!motorLeftFront.getMode().equals(DcMotor.RunMode.STOP_AND_RESET_ENCODER))
            resetEncoders();
    }
    public void SetEncoderPositionToRun()
    {
        motorLeftFront.setMode
                (DcMotor.RunMode.RUN_TO_POSITION
                );
        motorRightFront.setMode
                (DcMotor.RunMode.RUN_TO_POSITION
                );
        motorLeftBack.setMode
                (DcMotor.RunMode.RUN_TO_POSITION
                );
        motorRightBack.setMode
                (DcMotor.RunMode.RUN_TO_POSITION
                );
    }
    public void CheckBeaconForBlue( float speed, float waitTime)
    {
        float hsvValues[] = {0F,0F,0F};
        double startTime = runtime.time();
        setMotorSpeed(speed);
        Color.RGBToHSV((modernRobotics.red() * 255) / 800, (modernRobotics.green() * 255) / 800, (modernRobotics.blue() * 255) / 800, hsvValues);
        while(hsvValues[0]<150 && runtime.time() < startTime+waitTime)
        {
            Color.RGBToHSV((modernRobotics.red() * 255) / 800, (modernRobotics.green() * 255) / 800, (modernRobotics.blue() * 255) / 800, hsvValues);
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("startTime", startTime);
            telemetry.addData("timer", runtime.time());
            telemetry.update();
        }
        setMotorSpeed(0);
    }
    public void CheckBeaconForRed( float speed, float waitTime)
    {
        float hsvValues[] = {0F,0F,0F};
        double startTime = runtime.time();
        setMotorSpeed(speed);
        Color.RGBToHSV((modernRobotics.red() * 255) / 800, (modernRobotics.green() * 255) / 800, (modernRobotics.blue() * 255) / 800, hsvValues);
        while(hsvValues[0]>20 && runtime.time() < startTime+waitTime)
        {
            Color.RGBToHSV((modernRobotics.red() * 255) / 800, (modernRobotics.green() * 255) / 800, (modernRobotics.blue() * 255) / 800, hsvValues);
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("startTime", startTime);
            telemetry.addData("timer", runtime.time());
            telemetry.update();
        }
        setMotorSpeed(0);
    }
    public void turnBOR(int degres,float speed){turn(degres - MrGyro.getIntegratedZValue(),speed);}
    public void turn(int degrees, float speed) {
        int currentheading = -MrGyro.getIntegratedZValue();
        int tollerance = 1;
        double startTime = runtime.time();

        while(Math.abs(currentheading - degrees) > tollerance && runtime.time() < startTime +6) {
            currentheading = -MrGyro.getIntegratedZValue();
            if(Math.abs(degrees - currentheading) <25)
                speed  = .07f;
            telemetry.addData("check",Math.abs(currentheading - degrees));
            telemetry.addData("current heading", currentheading);
            telemetry.addData("degrees",degrees);
            telemetry.addData("speed", speed);
            telemetry.update();
            if(currentheading < degrees)
                turnRight(speed);
            else
                turnLeft(speed);
        }
        setMotorSpeed(0);
    }
    public void turnUsingLeftMotors(int degrees, float speed, float tollerance) {
        int currentheading = -MrGyro.getIntegratedZValue();
        double startTime = runtime.time();

        while(Math.abs(currentheading - degrees) > tollerance && runtime.time() < startTime +4) {
            currentheading = -MrGyro.getIntegratedZValue();
            if(Math.abs(degrees - currentheading) <25)
                speed  = .07f;
            telemetry.addData("check",Math.abs(currentheading - degrees));
            telemetry.addData("current heading", currentheading);
            telemetry.addData("degrees",degrees);
            telemetry.addData("speed", speed);
            telemetry.update();
            if(currentheading < degrees)
            {
                motorLeftFront.setPower(speed);
                motorLeftBack.setPower(speed);
            }
            else
            {
                motorLeftFront.setPower(-speed);
                motorLeftBack.setPower(-speed);
            }
        }
        setMotorSpeed(0);
    }
    public void turnUsingRightMotors(int degrees, float speed, float tollerance) {
        int currentheading = -MrGyro.getIntegratedZValue();
        double startTime = runtime.time();

        while(Math.abs(currentheading - degrees) > tollerance && runtime.time() < startTime +4) {
            currentheading = -MrGyro.getIntegratedZValue();
            if(Math.abs(degrees - currentheading) <25)
                speed  = .07f;
            telemetry.addData("check",Math.abs(currentheading - degrees));
            telemetry.addData("current heading", currentheading);
            telemetry.addData("degrees",degrees);
            telemetry.addData("speed", speed);
            telemetry.update();
            if(currentheading < degrees)
            {
                motorRightFront.setPower(-speed);
                motorRightBack.setPower(-speed);
            }
            else
            {
                motorRightFront.setPower(speed);
                motorRightBack.setPower(speed);
            }
        }
        setMotorSpeed(0);
    }

    public void searchForWhiteLine(float speed)
    {
        double startTime = runtime.time();
        setMotorSpeed(speed);
        while(OpticalDistance.getRawLightDetected() < .2
                && runtime.time() < startTime +4)
        {
            telemetry.addData("lightBack", OpticalDistance.getRawLightDetected());
        }
        setMotorSpeed(0);
    }

    public void turnLeft(float speed)
    {
        motorLeftFront.setPower(-speed);
        motorRightBack.setPower(speed);
        motorRightFront.setPower(speed);
        motorLeftBack.setPower(-speed);
    }

    public void turnRight(float speed)
    {
        motorLeftFront.setPower(speed);
        motorRightBack.setPower(-speed);
        motorRightFront.setPower(-speed);
        motorLeftBack.setPower(speed);
    }
}
