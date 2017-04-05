/**
 * Created by Mrinali on 3/24/2017.
 */

package org.firstinspires.ftc.teamcode.Mrinali;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.AllianceColor;

import ftclib.FtcDcMotor;
import ftclib.FtcOpMode;
import trclib.TrcDriveBase;

@Autonomous(name = "Mecanum Test 1", group = "Mecanum")
public class MecanumTest1 extends FtcOpMode {

    MecanumAuto auto = new MecanumAuto(this);

    @Override
    public void initRobot() {

    }

    @Override
    public void runOpMode() throws InterruptedException{

        auto.init(hardwareMap, telemetry, AllianceColor.RED);

        while (!isStarted()) {
            telemetry.addLine("Ready to Run");
            telemetry.addData("ODS Light", auto.odsLight.getLightDetected());
            telemetry.addData("ODS Light Raw", auto.odsLight.getRawLightDetected());
            telemetry.addData("Angle", auto.IMUheading());
            telemetry.addData("Distance", auto.getcmUltrasonic(auto.rangeSensor));
            telemetry.addData("Front left", auto.frontLeftMotor.getPosition());
            telemetry.addData("Front right", auto.frontRightMotor.getPosition());
            telemetry.addData("Back left", auto.backLeftMotor.getPosition());
            telemetry.addData("Back right", auto.backRightMotor.getPosition());
            telemetry.update();
        }

        auto.toWall();
        auto.toWhiteLine();
        auto.pushButton();
        auto.backup();
        auto.turn(0);
        auto.toWhiteLine();
        auto.pushButton();
    }

    public double IMUheading360() {
        return (auto.imu.getAngularOrientation().firstAngle + 360) %360;
    }
}
