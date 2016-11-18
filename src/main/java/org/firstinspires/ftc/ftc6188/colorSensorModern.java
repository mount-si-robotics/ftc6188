package org.firstinspires.ftc.ftc6188;
import com.qualcomm.hardware.ModernRoboticsI2cGyro;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by Wizard on 12/22/2015.
 */
public class colorSensorModern extends LinearOpMode{
    private ColorSensor modernRobotics;
    private GyroSensor gyroTest;
    private UltrasonicSensor ultra;
    private LegacyModule legacy;



    @Override
    public void runOpMode() throws InterruptedException {


        hardwareMap.logDevices();
        //gives the sensors a name in the config
        modernRobotics = hardwareMap.colorSensor.get("mr");
        gyroTest = hardwareMap.gyroSensor.get("gyro");
        ultra = hardwareMap.ultrasonicSensor.get("ultra");
        legacy = hardwareMap.legacyModule.get("legacy");

        legacy.enable9v(4, true);
        legacy.enable9v(5, true);
        //legacy.enableAnalogReadMode(5);

        gyroTest.calibrate();

        waitForStart();
        while(opModeIsActive())
        {
            //prints out the data from the sensors
            telemetry.addData("Red  ", modernRobotics.red());
            telemetry.addData("Green", modernRobotics.green());
            telemetry.addData("Blue ", modernRobotics.blue());
            telemetry.addData("4. h", gyroTest.getHeading());
            telemetry.addData("distance", ultra.getUltrasonicLevel());
			telemetry.update();
            waitOneFullHardwareCycle();


        }
    }
}
