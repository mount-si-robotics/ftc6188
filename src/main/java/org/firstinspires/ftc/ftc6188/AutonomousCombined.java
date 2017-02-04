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

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


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
public class AutonomousCombined extends LinearOpMode implements FtcMenu.MenuButtons {

    private ElapsedTime runtime = new ElapsedTime();
    public final float CIRCUMFENCE = (float)(4.00 * Math.PI);
    public final int ENCODERTICKS = 1120;
    public final float GEARRATIO = .5f;
    private int alliance = -1;
    boolean isFar = false;
    boolean shoot = false;
    boolean goCapBall = false;
    boolean parkCenter = false;
    boolean goBeacons = false;
    int delay = 0;
    private int PARALLELCLOSE = 35;

    private DcMotor motorLeftFront;
    private DcMotor motorLeftBack;
    private DcMotor motorRightFront;
    private DcMotor motorRightBack;
    private DcMotor ballLauncher;
    private DcMotor linSlide;
    private DcMotor lift1;
    private DcMotor lift2;

    private ModernRoboticsI2cRangeSensor USensor;

    private ColorSensor adafruitColor;

    private OpticalDistanceSensor optBack;
    private OpticalDistanceSensor optFront;

    private GyroSensor sensorType;
    private ModernRoboticsI2cGyro MrGyro;

    private DeviceInterfaceModule cdim;

    @Override
    public void runOpMode() {
        float hsvValues[] = {0F,0F,0F};
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorRightFront = hardwareMap.dcMotor.get("RFMotor");
        motorRightBack = hardwareMap.dcMotor.get("RBMotor");
        motorLeftFront = hardwareMap.dcMotor.get("LFMotor");
        motorLeftBack = hardwareMap.dcMotor.get("LBMotor");

        ballLauncher = hardwareMap.dcMotor.get("Launcher");
        linSlide = hardwareMap.dcMotor.get("linSlide");

        lift1 = hardwareMap.dcMotor.get("Lift1");
        lift2 = hardwareMap.dcMotor.get("Lift2");

        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);

        USensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "USensor");

        adafruitColor = hardwareMap.colorSensor.get("MRCSensor");

        optFront = hardwareMap.opticalDistanceSensor.get("ODSensorFront");
        optBack = hardwareMap.opticalDistanceSensor.get("ODSensorBack");

        sensorType = hardwareMap.gyroSensor.get("GSensor");

        cdim = hardwareMap.deviceInterfaceModule.get("cdim");

        MrGyro = (ModernRoboticsI2cGyro) sensorType;

        linSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runEncoders();
        turnOnLight();
        //ToDo: 1: select start pos, 2: shoot/dont shoot, 3: cap ball/ no cap ball, 4: park center/ramp, 5: beacons or balls

        MrGyro.calibrate();
        while(MrGyro.isCalibrating())
        {
        }
        //set up the robot depending on the situation
        FtcValueMenu menu = new FtcValueMenu("Alliance Blue: ", null, this, -1, 1, 2, -1, "%.2f");
        FtcValueMenu menu2 = new FtcValueMenu("Position Far: ", menu, this, -1,1,2,-1,"%.2f");
        menu2.setChildMenu(menu2);
        FtcValueMenu menu3 = new FtcValueMenu("Cap Ball: ", menu2, this, -1,1,2,-1,"%.2f");
        menu2.setChildMenu(menu3);
        FtcValueMenu menu4 = new FtcValueMenu("Beacons: ", menu3, this, -1,1,2,-1,"%.2f");
        menu2.setChildMenu(menu4);
        FtcValueMenu menu5 = new FtcValueMenu("Delay seconds: ", menu4, this, 0,15,.5,0, "%.2f");
        menu.setChildMenu(menu5);
        FtcMenu.setOpMode(this);
        FtcMenu.walkMenuTree(menu);
        alliance =(int) menu.getCurrentValue();
        delay = (int)(menu5.getCurrentValue() * 1000);
        if(menu2.getCurrentValue() == 1)
            isFar = true;
        if(menu3.getCurrentValue() == 1)
            goCapBall = true;
        if(menu4.getCurrentValue() == 1)
            goBeacons = true;

        while (!isStarted()) {
            telemetry.addData("Angle ", MrGyro.getHeading());
            telemetry.addData("Alliance ",alliance);
            telemetry.addData("Start Pos Far ", isFar);
            telemetry.addData("Cap Ball at End of Beacons", goCapBall);
            telemetry.addData("beacons over Cap at start",goBeacons);
            telemetry.addData("Delay ", delay/1000 + " seconds");
            telemetry.update();
        }
        runtime.reset();
        MrGyro.resetZAxisIntegrator();
        sleep(delay);
        /*
        if(!goBeacons)
        {
            moveRobot2(68,.2f);
        }
        else
         */
        //sets the angle to be parallel with the wall depending on alliance
        if(alliance == 1)
            PARALLELCLOSE = -PARALLELCLOSE;

        //move robot 67 inches at 40% power
        moveRobot2(67 * alliance,.3f);

        //turns robot parallel to wall
        if(alliance == 1)
            turnUsingLeftMotors(PARALLELCLOSE+2,.08f,0);
        else
            turnUsingRightMotors(PARALLELCLOSE,.08f,0);
        if (alliance == 1)
        {
            //searches for white line at 10% power with the front optical distance sensor
            searchForWhiteLine(-.1f * alliance, optFront);
            Color.RGBToHSV((adafruitColor.red() * 255) / 800, (adafruitColor.green() * 255) / 800, (adafruitColor.blue() * 255) / 800, hsvValues);
            sleep(500);
            Color.RGBToHSV((adafruitColor.red() * 255) / 800, (adafruitColor.green() * 255) / 800, (adafruitColor.blue() * 255) / 800, hsvValues);
            if(hsvValues[0] > 150 && hsvValues[0] < 320)
                pushButton();
            else
            {
                //searches for white line at 10% power with the back optical distance sensor
                searchForWhiteLine(.1f * alliance, optBack);
                pushButton();
            }
            Color.RGBToHSV((adafruitColor.red() * 255) / 800, (adafruitColor.green() * 255) / 800, (adafruitColor.blue() * 255) / 800, hsvValues);
            if (hsvValues[0] < 150 || hsvValues[0] > 320) {
                sleep(3700);
                pushButton();
            }
        }
        else
        {
            //searches for white line at 10% power with the front optical distance sensor
            searchForWhiteLine(-.1f * alliance, optFront);
            Color.RGBToHSV((adafruitColor.red() * 255) / 800, (adafruitColor.green() * 255) / 800, (adafruitColor.blue() * 255) / 800, hsvValues);
            DbgLog.msg("1 %.2f",hsvValues[0]);
            sleep(500);
            Color.RGBToHSV((adafruitColor.red() * 255) / 800, (adafruitColor.green() * 255) / 800, (adafruitColor.blue() * 255) / 800, hsvValues);
            DbgLog.msg("2 %.2f",hsvValues[0] );
            if(hsvValues[0] < 150 || hsvValues[0] > 320)
                pushButton();
            else
            {
                //searches for white line at 10% power with the baack optical distance sensor
                searchForWhiteLine(-.1f * alliance, optBack);
                pushButton();
            }
            Color.RGBToHSV((adafruitColor.red() * 255) / 800, (adafruitColor.green() * 255) / 800, (adafruitColor.blue() * 255) / 800, hsvValues);
            DbgLog.msg("3 %.2f",hsvValues[0]);
            if (hsvValues[0] > 150 && hsvValues[0] < 320)
            {
                sleep(3700);
                pushButton();
            }
        }
        //moves robot 30 inches at 30% power at parallel to wall
        moveRobot2(30 * alliance, .3f, PARALLELCLOSE);
        if (alliance == 1)
        {
            //searches for white line at 10% power with the front optical distance sensor
            searchForWhiteLine(.1f * alliance, optFront);
            Color.RGBToHSV((adafruitColor.red() * 255) / 800, (adafruitColor.green() * 255) / 800, (adafruitColor.blue() * 255) / 800, hsvValues);
            sleep(500);
            Color.RGBToHSV((adafruitColor.red() * 255) / 800, (adafruitColor.green() * 255) / 800, (adafruitColor.blue() * 255) / 800, hsvValues);
            if(hsvValues[0] > 150 && hsvValues[0] < 320)
                pushButton();
            else
            {
                //searches for white line at 10% power with the back optical distance sensor
                searchForWhiteLine(.1f * alliance, optBack);
                pushButton();
            }
            Color.RGBToHSV((adafruitColor.red() * 255) / 800, (adafruitColor.green() * 255) / 800, (adafruitColor.blue() * 255) / 800, hsvValues);
            if (hsvValues[0] < 150 || hsvValues[0] > 320) {
                sleep(3700);
                pushButton();
            }
        }

        else
        {
            //searches for white line at 10% power with the back optical distance sensor
            searchForWhiteLine(.1f * alliance, optBack);
            Color.RGBToHSV((adafruitColor.red() * 255) / 800, (adafruitColor.green() * 255) / 800, (adafruitColor.blue() * 255) / 800, hsvValues);
            sleep(500);
            Color.RGBToHSV((adafruitColor.red() * 255) / 800, (adafruitColor.green() * 255) / 800, (adafruitColor.blue() * 255) / 800, hsvValues);
            if(hsvValues[0] < 150 || hsvValues[0] > 320)
                pushButton();
            else
            {
                //searches for white line at 10% power with the front optical distance sensor
                searchForWhiteLine(.1f * alliance, optFront);
                pushButton();
            }
            Color.RGBToHSV((adafruitColor.red() * 255) / 800, (adafruitColor.green() * 255) / 800, (adafruitColor.blue() * 255) / 800, hsvValues);
            if (hsvValues[0] > 150 && hsvValues[0] < 320)
            {
                sleep(3700);
                pushButton();
            }
        }

        //moves robot 18 inches at 20% at 45 degrees
        //moveRobot2(18 * alliance,.2f,35);
        //turns robot to 0 degrees at 10%power
        //turn(0,.1f);
        /*if(goCapBall)
        {
            moveRobot2(60,.2f);
        }*/
        turnOffLight();

    }
    public void turnOffLight()
    {
        cdim.setDigitalChannelMode(7, DigitalChannelController.Mode.OUTPUT);
        cdim.setDigitalChannelState(7, false);

    }
    public void turnOnLight()
    {
        cdim.setDigitalChannelMode(7, DigitalChannelController.Mode.OUTPUT);
        cdim.setDigitalChannelState(7, true);

    }
    public void pushButton()
    {

        linSlide.setPower(-1f);
        sleep(1300);
        linSlide.setPower(1f);
        sleep(1300);
        linSlide.setPower(0);


    }

    public void moveRobot(double distance, float speed) {
        double ticksToInches = (ENCODERTICKS * GEARRATIO) / CIRCUMFENCE;
        int PositionTarget1 = motorLeftBack.getCurrentPosition() + (int) (distance * ticksToInches);
        int PositionTarget2 = motorRightFront.getCurrentPosition() + (int) (distance * ticksToInches);
        int PositionTarget3 = motorRightBack.getCurrentPosition() + (int) (distance * ticksToInches);
        int PositionTarget4 = motorLeftFront.getCurrentPosition() + (int) (distance * ticksToInches);
        int startTick = Math.abs(motorLeftFront.getCurrentPosition());

        motorLeftBack.setTargetPosition(PositionTarget1);
        motorRightFront.setTargetPosition(PositionTarget2);
        motorRightBack.setTargetPosition(PositionTarget3);
        motorLeftFront.setTargetPosition(PositionTarget4);

        SetEncoderPositionToRun();
        setMotorSpeed(speed / 4);
        while (motorLeftFront.isBusy() && motorLeftBack.isBusy()  && motorRightFront.isBusy()  && motorRightBack.isBusy()  && opModeIsActive()) {
            if(Math.abs(motorLeftFront.getCurrentPosition()) > startTick + 100 && Math.abs(motorLeftFront.getCurrentPosition()) < startTick + 200 )
                setMotorSpeed(speed / 3);
            else if(Math.abs(motorLeftFront.getCurrentPosition()) > startTick + 200 && Math.abs(motorLeftFront.getCurrentPosition()) < startTick + 300 )
                setMotorSpeed(speed / 2);
            else if(Math.abs(motorLeftFront.getCurrentPosition()) > startTick + 300)
                setMotorSpeed(speed);
            telemetry.addData("tickLeftFront", motorLeftFront.getCurrentPosition());
            telemetry.addData("tickLeftBack", motorLeftBack.getCurrentPosition());
            telemetry.addData("tickRightFront", motorRightFront.getCurrentPosition());
            telemetry.addData("tickRightBack", motorRightBack.getCurrentPosition());

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

        while (motorLeftFront.isBusy() && motorLeftBack.isBusy()  && motorRightFront.isBusy()  && motorRightBack.isBusy() &&  runtime.time() < startTime+6 && opModeIsActive()) {

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

        while (motorLeftFront.isBusy() &&  runtime.time() < startTime+6 && opModeIsActive()) {

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
    public void turnBOR(int degres,float speed){turn(degres - MrGyro.getIntegratedZValue(),speed);}
    public void turn(int degrees, float speed) {
        int currentheading = -MrGyro.getIntegratedZValue();
        int tollerance = 1;
        double startTime = runtime.time();

        while(Math.abs(currentheading - degrees) > tollerance && runtime.time() < startTime +4.5 && opModeIsActive()) {
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
    public void turnUsingRightMotors(int degrees, float speed, float tollerance) {
        int currentheading = -MrGyro.getIntegratedZValue();
        double startTime = runtime.time();

        while(Math.abs(currentheading - degrees) > tollerance && runtime.time() < startTime +4 && opModeIsActive()) {
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
    public void turnUsingLeftMotors(int degrees, float speed, float tollerance) {
        int currentheading = -MrGyro.getIntegratedZValue();
        double startTime = runtime.time();

        while(Math.abs(currentheading - degrees) > tollerance && runtime.time() < startTime +4 && opModeIsActive()) {
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


    public void searchForWhiteLine(float speed, OpticalDistanceSensor optSensor)
    {
        double startTime = runtime.time();
        setMotorSpeed(speed);
        while(optSensor.getRawLightDetected() < .5
                && runtime.time() < startTime +4 && opModeIsActive())
        {
            telemetry.addData("lightBack", optSensor.getRawLightDetected());
        }
        setMotorSpeed(0);
    }
        public void senseWall(float speed, int distance)
    {
        double startTime = runtime.time();
        setMotorSpeed(speed);

        while(USensor.cmUltrasonic() > distance && runtime.time() < startTime +10 && opModeIsActive())
        {

            telemetry.addData("distance ", USensor.cmUltrasonic());
            telemetry.update();
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

    public boolean isMenuUpButton() {
        return gamepad1.dpad_up;
    }

    @Override
    public boolean isMenuDownButton() {
        return gamepad1.dpad_down;
    }

    @Override
    public boolean isMenuEnterButton() {
        return gamepad1.dpad_right;
    }

    @Override
    public boolean isMenuBackButton() {
        return gamepad1.dpad_left;
    }
}
