package org.firstinspires.ftc.teamcode.Shashank.testcode;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.AllianceColor;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.AutoStateMachineBuilder;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.states.TurnState;

import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateName;

/**
 * Created by spmeg on 1/21/2017.
 */
@Autonomous(name = "TestStateMachineOp", group = "StateOps")
public class TestStateMachineOp extends OpMode {
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;

    private I2cDeviceSynchImpl rangeSensor = null;
    private I2cDevice rangeA;

    ElapsedTime runtime = new ElapsedTime();
    private LightSensor lightSensor;

    private ColorSensor leftColorSensor, rightColorSensor = null;

    enum S implements StateName {
        STATE,
        WAIT,
        STATE_2,
        WAIT_2,
        STATE_3,
        WAIT_3,
        STATE_4,
        WAIT_4,
        STOP
    };

    private StateMachine stateMachine;

    private AllianceColor beaconColor = null;

    private BNO055IMU imu;

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

        leftColorSensor  = hardwareMap.colorSensor.get("lcs");
        I2cAddr i2cAddr = I2cAddr.create8bit(0x4c);

        rightColorSensor = hardwareMap.colorSensor.get("rcs");
        rightColorSensor.setI2cAddress(i2cAddr);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        AutoStateMachineBuilder autoStateMachineBuilder = new AutoStateMachineBuilder(S.STATE);

        autoStateMachineBuilder.addEncoderDrive(leftMotor, rightMotor, S.STATE, S.WAIT, 7);
        autoStateMachineBuilder.addWait(S.WAIT, S.STATE_2, 3000);
        autoStateMachineBuilder.addEncoderDrive(leftMotor, rightMotor, S.STATE_2, S.WAIT_2, -7);
        autoStateMachineBuilder.addWait(S.WAIT_2, S.STATE_3, 3000);
        autoStateMachineBuilder.addTurn(leftMotor, rightMotor, S.STATE_3, S.WAIT_3, imu, 150);
        autoStateMachineBuilder.addWait(S.WAIT_3, S.STATE_4, 3000);
        autoStateMachineBuilder.addTurn(leftMotor, rightMotor, S.STATE_4, S.WAIT_4, imu, 0);
        autoStateMachineBuilder.addWait(S.WAIT_4, S.STOP, 3000);
        autoStateMachineBuilder.addStop(S.STOP);

        stateMachine = autoStateMachineBuilder.build();

        runtime.reset();
        telemetry.log().add("Finished init");
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        telemetry.log().add("Starting start method");
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
        telemetry.addData("left alpha", String.format("a=%d", leftColorSensor.alpha()));
        telemetry.addData("left red", String.format("r=%d", leftColorSensor.red()));
        telemetry.addData("left green", String.format("g=%d", leftColorSensor.green()));
        telemetry.addData("left blue", String.format("b=%d", leftColorSensor.blue()));
        telemetry.addData("right alpha", String.format("a=%d", rightColorSensor.alpha()));
        telemetry.addData("right red", String.format("r=%d", rightColorSensor.red()));
        telemetry.addData("right green", String.format("g=%d", rightColorSensor.green()));
        telemetry.addData("right blue", String.format("b=%d", rightColorSensor.blue()));
        telemetry.addData("left connec", leftColorSensor.getConnectionInfo());
        telemetry.addData("right connec", rightColorSensor.getConnectionInfo());
        telemetry.addData("imu ", imu.getAngularOrientation().firstAngle);
        telemetry.update();

        DbgLog.msg("CURRENT STATE " + stateMachine.getCurrentStateName());

        stateMachine.act();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
