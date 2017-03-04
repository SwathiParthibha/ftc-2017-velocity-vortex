package org.firstinspires.ftc.teamcode.Shashank.statemachine;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.os.AsyncTask;

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

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Sam.util.Util;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.states.TurnState;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.states.TurnStateEncoderDrive;

import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateName;

/**
 * Created by spmeg on 1/21/2017.
 */
@Autonomous(name = "StateMachineOp", group = "StateOps")
public class StateMachineOp extends OpMode {
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;

    private I2cDeviceSynchImpl rangeSensor = null;
    private I2cDevice rangeA;

    ElapsedTime runtime = new ElapsedTime();
    private LightSensor lightSensor;

    private ColorSensor leftColorSensor, rightColorSensor = null;
    private static boolean RUN = true;

    enum S implements StateName {
        GO_FORWARD,
        WAIT_BEFORE_TURN,
        FIRST_TURN,
        WAIT_TO_WHITE_LINE,
        TO_WHITE_LINE,
        DRIVE,
        WHITE_LINE_PIVOT_WAIT,
        PIVOT_TO_LINE,
        FOLLOW_lINE,
        PRESS_BEACON,
        GO_BACKWARD,
        SECOND_TURN,
        _2ND_TO_WHITE_LINE,
        _2ND_DRIVE,
        _2ND_WHITE_LINE_PIVOT_WAIT,
        _2ND_PIVOT_TO_LINE,
        _2ND_FOLLOW_lINE,
        _2ND_PRESS_BEACON,
        SECOND_GO_BACKWARD,
        THIRD_TURN,
        SECOND_GO_FORWARD,
        STOP
    };

    private StateMachine stateMachine;

    private AllianceColor beaconColor = null;

    private BNO055IMU imu;

    @Override
    public void init() {
        AsyncTask.execute(new Runnable() {
            @Override
            public void run() {
                Activity activity = (Activity) hardwareMap.appContext;
                activity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        AlertDialog.Builder builder = new AlertDialog.Builder(hardwareMap.appContext);
                        builder.setTitle(R.string.pickColor)
                                .setItems(R.array.allianceColor, new DialogInterface.OnClickListener() {
                                    public void onClick(DialogInterface dialog, int itemPos) {
                                        // The 'which' argument contains the index position
                                        // of the selected item
                                        if(itemPos == 0){
                                            beaconColor = AllianceColor.BLUE;
                                        } else {
                                            beaconColor = AllianceColor.RED;
                                        }
                                    }
                                });
                        AlertDialog alertDialog = builder.create();
                        telemetry.log().add("alert dialog created");
                        telemetry.update();
                        alertDialog.show();
                    }
                });

            }
        });

        AsyncTask.execute(new Runnable() {
            @Override
            public void run() {
                Activity activity = (Activity) hardwareMap.appContext;
                activity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        AlertDialog.Builder builder = new AlertDialog.Builder(hardwareMap.appContext);
                        builder.setTitle(R.string.pickColor)
                                .setMessage("Run the Opmdode")
                                .setPositiveButton("YES", new DialogInterface.OnClickListener() {
                                    @Override
                                    public void onClick(DialogInterface dialog, int which) {
                                        RUN = true;
                                    }
                                })
                                .setNegativeButton("NO", new DialogInterface.OnClickListener() {
                                    @Override
                                    public void onClick(DialogInterface dialog, int which) {
                                        RUN = false;
                                    }
                                });

                        AlertDialog alertDialog = builder.create();
                        telemetry.log().add("alert dialog created");
                        telemetry.update();
                        alertDialog.show();
                    }
                });

            }
        });

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

        DbgLog.msg("SYSTEM STATUS OF IMU BEFORE INIT " + imu.getSystemStatus().toString());

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        DbgLog.msg("SYSTEM STATUS OF IMU after INIT " + imu.getSystemStatus().toString());

        DbgLog.msg("FINISHED PICKING ALLIANCE COLOR " + beaconColor);
        DbgLog.msg("IMU INITIALIZED " + imu.isGyroCalibrated());

        if(beaconColor == AllianceColor.RED)
            telemetry.log().add("Beacon color is: RED");
        else if(beaconColor == AllianceColor.BLUE)
            telemetry.log().add("Beacon color is: BLUE");
        else if(beaconColor == null)
            telemetry.log().add("Beacon color is: NULL");

        runtime.reset();
        telemetry.log().add("Finished init");
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        telemetry.log().add("Starting start method");
        telemetry.update();

        while (beaconColor  == null){
            Util.waitUntil(50);
        }

        int firstTurnAngle = 30;
        int thirdTurnAngle = 124;

        if(beaconColor == AllianceColor.RED){
            firstTurnAngle = 360 - firstTurnAngle;
            thirdTurnAngle = 360 - thirdTurnAngle;
        }

        AutoStateMachineBuilder autoStateMachineBuilder = new AutoStateMachineBuilder(S.GO_FORWARD);

        //go forward a little
        autoStateMachineBuilder.addEncoderDrive(leftMotor, rightMotor, S.GO_FORWARD, S.FIRST_TURN, 3);

        //turn to designated degree
        autoStateMachineBuilder.addTurn(leftMotor, rightMotor, S.FIRST_TURN, S.WAIT_TO_WHITE_LINE, imu, firstTurnAngle);

        //autoStateMachineBuilder.addEncoderDrive(leftMotor, rightMotor, S.ENCODER_TO_WHITE_LINE, S.TO_WHITE_LINE, 18);
        autoStateMachineBuilder.addWait(S.WAIT_TO_WHITE_LINE, S.TO_WHITE_LINE, 500);
        //go to the white line
        autoStateMachineBuilder.addToWhiteLine(S.TO_WHITE_LINE, S.WHITE_LINE_PIVOT_WAIT, leftMotor, rightMotor, lightSensor);

        //wait for a little bit
        autoStateMachineBuilder.addWait(S.WHITE_LINE_PIVOT_WAIT, S.PIVOT_TO_LINE, 500);

        //pivot to the line
        autoStateMachineBuilder.addPivotToWhiteLine(leftMotor, rightMotor, lightSensor, S.PIVOT_TO_LINE, S.FOLLOW_lINE, beaconColor);

        //line follow to the beacon
        autoStateMachineBuilder.addLineFollow(telemetry, S.FOLLOW_lINE, S.PRESS_BEACON, leftMotor, rightMotor, lightSensor, rangeSensor, beaconColor);

        //press the beacon
        //autoStateMachineBuilder.addPressBeacon(telemetry, S.PRESS_BEACON, S.GO_BACKWARD, leftMotor, rightMotor, leftColorSensor, rightColorSensor, beaconColor);
        autoStateMachineBuilder.addPressBeacon(telemetry, S.PRESS_BEACON, S.STOP, leftMotor, rightMotor, leftColorSensor, rightColorSensor, beaconColor);
        //go backward
        /*autoStateMachineBuilder.addEncoderDrive(leftMotor, rightMotor, S.GO_BACKWARD, S.SECOND_TURN, 0);

        //turn to 0 degrees
        autoStateMachineBuilder.addTurn(leftMotor, rightMotor, S.SECOND_TURN, S._2ND_TO_WHITE_LINE, imu, 0);

        //repeat
        autoStateMachineBuilder.addToWhiteLine(S._2ND_TO_WHITE_LINE, S._2ND_DRIVE, leftMotor, rightMotor, lightSensor);
        autoStateMachineBuilder.addEncoderDrive(leftMotor, rightMotor, S._2ND_DRIVE, S._2ND_WHITE_LINE_PIVOT_WAIT, 0.5);
        autoStateMachineBuilder.addWait(S._2ND_WHITE_LINE_PIVOT_WAIT, S._2ND_PIVOT_TO_LINE, 300);
        autoStateMachineBuilder.addPivotToWhiteLine(leftMotor, rightMotor, lightSensor, S._2ND_PIVOT_TO_LINE, S._2ND_FOLLOW_lINE, beaconColor);
        autoStateMachineBuilder.addLineFollow(telemetry, S._2ND_FOLLOW_lINE, S._2ND_PRESS_BEACON, leftMotor, rightMotor, lightSensor, rangeSensor, beaconColor);
        autoStateMachineBuilder.addPressBeacon(telemetry, S._2ND_PRESS_BEACON, S.SECOND_GO_BACKWARD, leftMotor, rightMotor, leftColorSensor, rightColorSensor, beaconColor);
        autoStateMachineBuilder.addEncoderDrive(leftMotor, rightMotor, S.SECOND_GO_BACKWARD, S.THIRD_TURN, 0);
        autoStateMachineBuilder.addTurn(leftMotor, rightMotor, S.THIRD_TURN, S.SECOND_GO_BACKWARD, imu, thirdTurnAngle);
        autoStateMachineBuilder.addEncoderDrive(leftMotor, rightMotor, S.SECOND_GO_BACKWARD, S.STOP, 36);*/
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
        telemetry.addData("IMU", imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle);
        telemetry.addData("IMU", imu.getAngularOrientation().firstAngle);
        telemetry.addData("IMU SYSTEM STATUS", imu.getSystemStatus().toString());
        telemetry.update();

        if(RUN)
            stateMachine.act();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
