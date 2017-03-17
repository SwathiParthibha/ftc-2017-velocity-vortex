package org.firstinspires.ftc.teamcode.Shashank.autonomous;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.os.AsyncTask;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Sam.util.Util;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.AllianceColor;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.AutoStateMachineBuilder;

import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateName;

/**
 * Created by spmeg on 1/21/2017.
 */
@Autonomous(name = "GoStraightMachineOp", group = "StateOps")
@Disabled
public class GoStraightMachineOp extends OpMode {
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;

    ElapsedTime runtime = new ElapsedTime();

    enum S implements StateName {
        WAIT,
        GO_STRAIGHT,
        SHOOT,
        SECOND_GO_STRAIGHT,
        STOP
    };

    private StateMachine stateMachine;

    private DcMotor scooper;
    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor sweeper;
    private Servo leftArm;
    private Servo rightArm;
    private Servo capArm;

    @Override
    public void init() {
        leftMotor = this.hardwareMap.dcMotor.get("l");
        rightMotor = this.hardwareMap.dcMotor.get("r");
        scooper = this.hardwareMap.dcMotor.get("scooper");
        shooter1 = this.hardwareMap.dcMotor.get("shooter1");
        shooter2 = this.hardwareMap.dcMotor.get("shooter2");
        sweeper = this.hardwareMap.dcMotor.get("sweeper");
        leftArm = this.hardwareMap.servo.get("leftservo");
        rightArm = this.hardwareMap.servo.get("rightservo");
        capArm = this.hardwareMap.servo.get("capArm");

        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        runtime.reset();
        telemetry.log().add("Finished init");
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        telemetry.log().add("Starting start method");
        telemetry.update();

        AutoStateMachineBuilder autoStateMachineBuilder = new AutoStateMachineBuilder(S.GO_STRAIGHT);

        //go forward a little
        autoStateMachineBuilder.addWait(S.WAIT, S.GO_STRAIGHT, 20000);
        autoStateMachineBuilder.addEncoderDrive(leftMotor, rightMotor, S.GO_STRAIGHT, S.SHOOT, 15);
        autoStateMachineBuilder.addShoot(S.SHOOT,  S.SECOND_GO_STRAIGHT, scooper, shooter1, shooter2, sweeper, leftArm, rightArm);
        autoStateMachineBuilder.addEncoderDrive(leftMotor, rightMotor, S.SECOND_GO_STRAIGHT, S.STOP, 18);
        autoStateMachineBuilder.addStop(S.STOP);

        stateMachine = autoStateMachineBuilder.build();

        telemetry.log().add("Finished start");
        telemetry.update();
    }

    @Override
    public void loop() {
        if(stateMachine == null)
            telemetry.log().add("statemachine is null");

        telemetry.addData("State", stateMachine.getCurrentStateName());
        telemetry.update();

        stateMachine.act();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
