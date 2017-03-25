package org.firstinspires.ftc.teamcode.Mrinali;

/**
 * Created by Mrinali on 3/24/2017.
 */

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.Sam.shooter.MotorFactory;
import org.firstinspires.ftc.teamcode.Sam.shooter.RPMThreadMilliseconds;
import org.firstinspires.ftc.teamcode.Sam.shooter.beans.ShooterMotor;
import org.firstinspires.ftc.teamcode.Sam.shooter.power.PowerManager;
import org.firstinspires.ftc.teamcode.Sam.shooter.util.Constants;
import org.firstinspires.ftc.teamcode.Sam.util.Util;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.AllianceColor;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.AutoStateMachineBuilder;

import java.util.concurrent.Executor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateName;

@Autonomous(name="Mecanum Test", group="Mecanum")
public class MecanumAutoTest extends LinearOpMode{

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    @Override
    public void runOpMode() throws InterruptedException{

        frontLeftMotor = hardwareMap.dcMotor.get("motor_3");
        frontRightMotor = hardwareMap.dcMotor.get("motor_2");
        backLeftMotor = hardwareMap.dcMotor.get("motor_1");
        backRightMotor = hardwareMap.dcMotor.get("motor_4");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        while (!isStarted()) {
            telemetry.addLine("Ready to Run");
            telemetry.update();
        }

        /* Straight forward
        telemetry.log().add("All motors forward");
        frontLeftMotor.setPower(.5);
        frontRightMotor.setPower(.5);
        backLeftMotor.setPower(.5);
        backRightMotor.setPower(.5);
        */

        /* diagonal front right
        telemetry.log().add("Front left and back right");
        frontLeftMotor.setPower(.5);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(.5);
        sleep(2000);
        */

        /* right */
        telemetry.log().add("Front left, back right forward");
        telemetry.log().add("Front right, back left backward");
        frontLeftMotor.setPower(.5);
        frontRightMotor.setPower(-.5);
        backLeftMotor.setPower(-.5);
        backRightMotor.setPower(.5);
        sleep(2000);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
