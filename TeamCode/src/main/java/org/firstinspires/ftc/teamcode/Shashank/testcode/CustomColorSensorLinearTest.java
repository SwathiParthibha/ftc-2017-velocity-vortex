package org.firstinspires.ftc.teamcode.Shashank.testcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

/**
 * Created by spmeg on 10/8/2016.
 */
@Autonomous(name = "CustomColorSensorLinearTest", group = "Test")
@Disabled
public class CustomColorSensorLinearTest extends LinearOpMode {
    ColorSensor colorSensor;
    DeviceInterfaceModule dim;

    @Override
    public void runOpMode() throws InterruptedException
    {
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        colorSensor  = hardwareMap.colorSensor.get("lcs");

        colorSensor.enableLed(true);
        // possibly change some (notably gain and / or integration time), then
        // leftColorSensor.initialize(params);

        // possibly change some (notably gain and / or integration time), then
        // colorSensor.initialize(params);



        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("right", String.format("a=%d r=%d g=%d b=%d", colorSensor.alpha(), colorSensor.red(), colorSensor.green(), colorSensor.blue()));
            telemetry.addData("Color Sensor alpha", colorSensor.alpha());
            telemetry.addData("Color Sensor red", colorSensor.red());
            telemetry.addData("Color Sensor blue", colorSensor.blue());
            telemetry.addData("Color Sensor green", colorSensor.green());
            this.updateTelemetry(telemetry);

            Thread.sleep(500);
        }

    }
}

