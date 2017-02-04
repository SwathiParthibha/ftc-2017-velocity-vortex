package org.firstinspires.ftc.teamcode.Shashank.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Shashank.utils.RangeSensorRunnable;
import org.firstinspires.ftc.teamcode.Shashank.utils.ThreadSharedObject;

/**
 * Created by spmeg on 1/17/2017.
 */
@Autonomous(name = "ThreadedLineFollowingOpmode", group = "custom examples")
@Disabled
public class ThreadedLineFollowingOpmode extends OpMode {
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;

    private I2cDeviceSynchImpl rangeSensor = null;
    private byte[] rangeSensorCache;
    private I2cDevice rangeA;

    private RangeSensorRunnable rangeSensorRunnable = null;

    private ThreadSharedObject threadSharedObject = new ThreadSharedObject();

    ElapsedTime runtime = new ElapsedTime();
    private LightSensor lightSensor;

    private Thread testThread = null;

    @Override
    public void init() {
        leftMotor = this.hardwareMap.dcMotor.get("l");
        rightMotor = this.hardwareMap.dcMotor.get("r");

        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rangeA = hardwareMap.i2cDevice.get("range sensor");// Primary range sensor
        rangeSensor = new I2cDeviceSynchImpl(rangeA, I2cAddr.create8bit(0x2a), false);
        rangeSensor.engage();

        lightSensor = hardwareMap.lightSensor.get("light sensor");
        lightSensor.enableLed(true);

        runtime.reset();
        telemetry.log().add("Finished init");
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        telemetry.log().add("Starting stat method");
        telemetry.update();

        runtime.reset();
        rangeSensorRunnable = new RangeSensorRunnable(this.telemetry, runtime, rangeSensor, threadSharedObject);
        testThread = new Thread(rangeSensorRunnable);

        testThread.start();

        telemetry.log().add("Finished start");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Range Sensor ultrasonic from thread", threadSharedObject.getInteger(rangeSensorRunnable.getULTRASONIC_KEY()));
        telemetry.update();

        if(threadSharedObject.getInteger(rangeSensorRunnable.getULTRASONIC_KEY()) > 11){
            if(lightSensor.getLightDetected() > 0.3){
                leftMotor.setPower(0.2);
                rightMotor.setPower(0);
            } else {
                leftMotor.setPower(0);
                rightMotor.setPower(0.2);
            }
        } else {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }

    @Override
    public void stop() {
        super.stop();
        rangeSensorRunnable.requestStop();
    }
}
