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

@Autonomous(name = "Autonomous")
//@Disabled
public class AutonomousTestThomas extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    public final float CIRCUMFENCE = (float)(4.00 * Math.PI);
    public final int ENCODERTICKS = 1120;
    public final float GEARRATIO = .5f;

    // DcMotor leftMotor = null;
    // DcMotor rightMotor = null;

    private DcMotor motorLeftFront;
    private DcMotor motorLeftBack;
    private DcMotor motorRightFront;
    private DcMotor motorRightBack;

    private Servo buttonPusher;

    private ColorSensor modernRobotics;
    private OpticalDistanceSensor OpticalDistance;
    private GyroSensor MrGyro;

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
        MrGyro = hardwareMap.gyroSensor.get("GSensor");

        buttonPusher.setPosition(0);

        resetEncoders();
        checkEncoder();
        runEncoders();

        MrGyro.calibrate();

        // Wait for the game to start (driver presses PLAY)
        //move to side
        //search for white line
        //search for what color is
        //determine to drop buttonpusher now or later
        /*

        moveRobot(distance,speed);
        searchForWhiteline(speed,light);
        either if(CheckBeaconForBlue()) or if(CheckBeaconForRed());
        decide to moveRobot(); or buttonPusher.setPosition(position);
        buttonPusher.setPosition(0);
        moveRobot(distance,speed);
        searchForWhiteline(speed,light);
        either if(CheckBeaconForBlue()) or if(CheckBeaconForRed());
        decide to moveRobot(); or buttonPusher.setPosition(position);*/



        waitForStart();
        runtime.reset();

        sleep(10000);
        moveRobot(62,.6f);



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
        // Turn On RUN_TO_POSITION
        SetEncoderPositionToRun();
        setMotorSpeed(speed);

        while (motorLeftFront.isBusy()) {
            telemetry.addData("Path2",
                    motorLeftFront.getCurrentPosition());

            telemetry.update();
        }
        setMotorSpeed(0);
        runEncoders();
        /*runEncoders();
        checkEncoder();
        runEncoders();
        double tTicks = (distance / (CIRCUMFENCE / ENCODERTICKS));
        while(Math.abs(motorLeftFront.getCurrentPosition()) < tTicks) {


            telemetry.addData("left front", motorLeftFront.getCurrentPosition());
            telemetry.addData("right front", motorLeftFront.getCurrentPosition());
            telemetry.addData("left back", motorLeftBack.getCurrentPosition());
            telemetry.addData("right back", motorRightBack.getCurrentPosition());
            runEncoders();
            setMotorSpeed(speed);
            telemetry.update();
        }
        resetMotor();
        resetEncoders();*/
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
    //turn off the encoders
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
    //increase the ticks on the encoders
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
    public void resetMotor()
    {
        setMotorSpeed(0);
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
    /*public boolean CheckBeaconForBlue(float red, float green, float blue, float speed)
    {

        return modernRobotics.red() < modernRobotics.blue();

    }*/
    /*public boolean CheckBeaconForRed(float red, float green, float blue, float speed)
    {

        return modernRobotics.blue() < modernRobotics.red();

    }*/
   /* public void searchForWhiteline(float speed,float light)
    {
        setMotorSpeed(speed);
        while(OpticalDistance.getLightDetected() < light)
        {
            telemetry.addData("light returned", OpticalDistance.getLightDetected());
        }
        telemetry.update();
        setMotorSpeed(0);
    }*/

    public void turn(int degrees, float maxSpeed, float minimumTurnSpeed)
    {
        float speed = 0.0f;
        float precision = 0.1f;
        boolean leftTurn = false;
        boolean rightTurn = false;
        int newGyro = MrGyro.getHeading();
        float degrees_needed = Math.abs(degrees - newGyro);
        float numeratorSpeed;
        if(degrees_needed > 180 && newGyro < 180) {
            leftTurn = true;
            degrees_needed = Math.abs(degrees_needed - 360);
        }
        else if(degrees_needed > 180) {
            rightTurn = true;
            degrees_needed = Math.abs(degrees_needed - 360);
        }

        while (newGyro !=degrees && runtime.time() < 29.5f) {
            newGyro = MrGyro.getHeading();

            if(leftTurn && newGyro < 180) {
                numeratorSpeed = Math.abs(degrees - newGyro - 360);
                newGyro += 360;
            }
            else if(rightTurn && newGyro > 180)
            {
                numeratorSpeed = Math.abs(degrees - newGyro + 360);
                newGyro -= 360;
            }
            else
                numeratorSpeed = Math.abs(degrees - newGyro);

            speed = maxSpeed * (numeratorSpeed / degrees_needed);

            if (speed > 1.0)
                speed = 1.0f;
            if (speed < minimumTurnSpeed)
                speed = minimumTurnSpeed;
            telemetry.addData("4. h", newGyro);
            telemetry.addData("speed", speed);
            telemetry.update();
            if (newGyro > degrees + precision && newGyro < degrees - precision) {
                resetMotor();
                break;
            } else if (newGyro < degrees) {
                // turn right
                turnRight(speed);
            } else {
                // turn left
                turnLeft(speed);
            }

        }

    }
    public void turnLeft(float speed)
    {
        stopEncoders();
        motorLeftFront.setPower(-speed);
        motorRightBack.setPower(speed);
        motorRightFront.setPower(speed);
        motorLeftBack.setPower(-speed);
    }
    //turns robot right
    public void turnRight(float speed)
    {
        stopEncoders();
        motorLeftFront.setPower(speed);
        motorRightBack.setPower(-speed);
        motorRightFront.setPower(-speed);
        motorLeftBack.setPower(speed);
    }
}
