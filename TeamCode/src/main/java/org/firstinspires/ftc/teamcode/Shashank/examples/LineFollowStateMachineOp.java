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

import org.firstinspires.ftc.teamcode.Shashank.statemachine.AutoStateMachineBuilder;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.BeaconColor;
import org.firstinspires.ftc.teamcode.Shashank.utils.RangeSensorRunnable;
import org.firstinspires.ftc.teamcode.Shashank.utils.ThreadSharedObject;

import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateName;

/**
 * Created by spmeg on 1/21/2017.
 */
@Autonomous(name = "LineFollowStateMachineOp", group = "StateOps")
@Disabled
public class LineFollowStateMachineOp extends OpMode {
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;

    private I2cDeviceSynchImpl rangeSensor = null;
    private byte[] rangeSensorCache;
    private I2cDevice rangeA;

    ElapsedTime runtime = new ElapsedTime();
    private LightSensor lightSensor;

    private Thread testThread = null;

    enum S implements StateName {
        FOLLOW_lINE,
        WAIT,
        STOP
    };

    private StateMachine stateMachine;

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

        AutoStateMachineBuilder autoStateMachineBuilder = new AutoStateMachineBuilder(S.WAIT);

        autoStateMachineBuilder.addWait(S.WAIT, S.FOLLOW_lINE, 3000);
        autoStateMachineBuilder.addLineFollow(telemetry, S.FOLLOW_lINE, S.STOP, leftMotor, rightMotor, lightSensor, rangeSensor, BeaconColor.BLUE);
        autoStateMachineBuilder.addStop(S.STOP);

        stateMachine = autoStateMachineBuilder.build();

        runtime.reset();
        telemetry.log().add("Finished init");
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        telemetry.log().add("Starting stat method");
        telemetry.update();

        telemetry.log().add("Finished start");
        telemetry.update();
    }

    @Override
    public void loop() {
        if(stateMachine == null)
            telemetry.log().add("statemachine is null");

        telemetry.addData("State", stateMachine.getCurrentStateName());
        telemetry.addData("Light detected", lightSensor.getLightDetected());
        telemetry.update();
        stateMachine.act();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
