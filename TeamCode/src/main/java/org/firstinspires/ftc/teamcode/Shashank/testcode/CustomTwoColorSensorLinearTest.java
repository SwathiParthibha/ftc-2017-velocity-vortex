package org.firstinspires.ftc.teamcode.Shashank.testcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by spmega4567 on 10/7/16.
 */
@Autonomous(name = "CustomTwoColorSensorLinearTest", group = "Tests")
public class CustomTwoColorSensorLinearTest extends LinearOpMode
{
    ColorSensor leftColorSensor;
    ColorSensor rightColorSensor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftColorSensor  = hardwareMap.colorSensor.get("lcs");
        I2cAddr i2cAddr = I2cAddr.create8bit(0x4c);


        rightColorSensor = hardwareMap.colorSensor.get("rcs");
        rightColorSensor.setI2cAddress(i2cAddr);

        waitForStart();

        while (opModeIsActive())
        {
            int left = leftColorSensor.argb();
            int right = rightColorSensor.argb();
            telemetry.addData("left alpha", String.format("a=%d", leftColorSensor.alpha()));
            telemetry.addData("left red", String.format("r=%d", leftColorSensor.red()));
            telemetry.addData("left green", String.format("g=%d", leftColorSensor.green()));
            telemetry.addData("left blue", String.format("b=%d", leftColorSensor.blue()));
            telemetry.addData("right alpha", String.format("a=%d", rightColorSensor.alpha()));
            telemetry.addData("right red", String.format("r=%d", rightColorSensor.red()));
            telemetry.addData("right green", String.format("g=%d", rightColorSensor.green()));
            telemetry.addData("right blue", String.format("b=%d", rightColorSensor.blue()));
            telemetry.update();

            Thread.sleep(500);
        }

    }


}
