package org.firstinspires.ftc.ftc6188;



import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Wizard on 12/22/2015.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="SensorTester", group="Iterative Opmode")
public class colorSensorModern extends LinearOpMode{
    private ColorSensor adafruitSensor;
    private GyroSensor gyroTest;
    private OpticalDistanceSensor optBack;
    private OpticalDistanceSensor optFront;
    private ModernRoboticsI2cRangeSensor USensor;

    public void runOpMode(){

        float hsvValues[] = {0F,0F,0F};
       hardwareMap.logDevices();
        adafruitSensor = hardwareMap.colorSensor.get("mr2");
        gyroTest = hardwareMap.gyroSensor.get("gyro");
        optFront = hardwareMap.opticalDistanceSensor.get("ODSensorFront");
        optBack = hardwareMap.opticalDistanceSensor.get("ODSensorBack");
        USensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultra");


        gyroTest.calibrate();
        waitForStart();
        while(opModeIsActive())
        {

            Color.RGBToHSV((adafruitSensor.red() * 255) / 800, (adafruitSensor.green() * 255) / 800, (adafruitSensor.blue() * 255) / 800, hsvValues);
            telemetry.addData("Hue: ", hsvValues[0]);
            telemetry.addData("distance cm: ",USensor.getDistance(DistanceUnit.CM));
            telemetry.addData("4. h: ", gyroTest.getHeading());
            telemetry.addData("Raw Light front: ", optFront.getRawLightDetected());
            telemetry.addData("Raw Light Back: ", optBack.getRawLightDetected());

            telemetry.update();

        }
    }
}
