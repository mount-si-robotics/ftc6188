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
    private GyroSensor gyroTest;
    private ModernRoboticsI2cGyro mrGyro;
   // private UltrasonicSensor ultra;
    //private LegacyModule legacy;
    private OpticalDistanceSensor ODSensor;




    public void runOpMode(){

        float hsvValues[] = {0F,0F,0F};
       hardwareMap.logDevices();
        //gives the sensors a name in the config
        modernRobotics = hardwareMap.colorSensor.get("mr");
        gyroTest = hardwareMap.gyroSensor.get("gyro");
        mrGyro = (ModernRoboticsI2cGyro) gyroTest;
        //ultra = hardwareMap.ultrasonicSensor.get("ultra");
        //legacy = hardwareMap.legacyModule.get("legacy");
        ODSensor = hardwareMap.opticalDistanceSensor.get("ODSensor");

        //legacy.enable9v(4, true);
        //legacy.enable9v(5, true);
        //legacy.enableAnalogReadMode(5);

        gyroTest.calibrate();


        waitForStart();
        while(opModeIsActive())
        {

            Color.RGBToHSV((modernRobotics.red() * 255) / 800, (modernRobotics.green() * 255) / 800, (modernRobotics.blue() * 255) / 800, hsvValues);
            //prints out the data from the sensors
            telemetry.addData("Red  ", modernRobotics.red());
            telemetry.addData("Green", modernRobotics.green());
            telemetry.addData("Blue ", modernRobotics.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("sat", hsvValues[1]);
            telemetry.addData("value", hsvValues[2]);
            telemetry.addData("intergratedZValue", mrGyro.getIntegratedZValue());
            telemetry.addData("4. h", gyroTest.getHeading());
            //telemetry.addData("distance", ultra.getUltrasonicLevel());
            telemetry.addData("Light Back ", ODSensor.getLightDetected());
            telemetry.addData("Raw Light Back", ODSensor.getRawLightDetected());
            telemetry.update();



        }
    }
}
