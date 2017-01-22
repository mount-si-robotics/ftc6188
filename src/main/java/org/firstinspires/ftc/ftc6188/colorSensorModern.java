package org.firstinspires.ftc.ftc6188;



import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by Wizard on 12/22/2015.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="SensorTester", group="Iterative Opmode")
public class colorSensorModern extends LinearOpMode{
    private ColorSensor modernRobotics;
    private ColorSensor adafruitSensor;
    private GyroSensor gyroTest;
    private ModernRoboticsI2cGyro mrGyro;
    private OpticalDistanceSensor ODSensor;
    private UltrasonicSensor USensor;
    private LegacyModule legacy;

    public void runOpMode(){

        float hsvValues[] = {0F,0F,0F};
        float hsvValues2[] = {0F,0F,0F};
       hardwareMap.logDevices();
        modernRobotics = hardwareMap.colorSensor.get("mr");
        adafruitSensor = hardwareMap.colorSensor.get("mr2");
        gyroTest = hardwareMap.gyroSensor.get("gyro");
        mrGyro = (ModernRoboticsI2cGyro) gyroTest;
        ODSensor = hardwareMap.opticalDistanceSensor.get("ODSensor");
        USensor = hardwareMap.ultrasonicSensor.get("ultra");
        legacy = hardwareMap.legacyModule.get("legacy");

        legacy.enable9v(4,true);

        gyroTest.calibrate();
        modernRobotics.enableLed(true);
        waitForStart();
        while(opModeIsActive())
        {
            Color.RGBToHSV(modernRobotics.red() * 8, modernRobotics.green() * 8, modernRobotics.blue() * 8, hsvValues);

            Color.RGBToHSV((adafruitSensor.red() * 255) / 800, (adafruitSensor.green() * 255) / 800, (adafruitSensor.blue() * 255) / 800, hsvValues2);
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("Hue2", hsvValues2[0]);
            telemetry.addData("distance cm:",USensor.getUltrasonicLevel());
            telemetry.addData("4. h", gyroTest.getHeading());
            telemetry.addData("Raw Light Back", ODSensor.getRawLightDetected());
            telemetry.update();

        }
    }
}
