package org.firstinspires.ftc.teamcode.Shashank.statemachine;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.os.Looper;
import android.view.View;

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
    private byte[] rangeSensorCache;
    private I2cDevice rangeA;

    ElapsedTime runtime = new ElapsedTime();
    private LightSensor lightSensor;

    private Thread testThread = null;

    private ColorSensor leftColorSensor, rightColorSensor = null;

    enum S implements StateName {
        TO_WHITE_LINE,
        FOLLOW_lINE,
        PRESS_BEACON,
        WAIT,
        _2ND_TO_WHITE_LINE,
        _2ND_FOLLOW_lINE,
        _2ND_PRESS_BEACON,
        _2ND_WAIT,
        STOP
    };

    private StateMachine stateMachine;

    private BeaconColor beaconColor = null;

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
        leftColorSensor.setI2cAddress(i2cAddr);

        rightColorSensor = hardwareMap.colorSensor.get("rcs");

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
                                    beaconColor = BeaconColor.BLUE;
                                } else {
                                    beaconColor = BeaconColor.RED;
                                }
                            }
                        });
                AlertDialog alertDialog = builder.create();
                telemetry.log().add("alert dialog created");
                telemetry.update();
                alertDialog.show();
            }
        });

        //wait for the user to set the color
        while (beaconColor == null){
            try {
                Thread.sleep(150);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        if(beaconColor == BeaconColor.RED)
            telemetry.log().add("Beacon color is: RED");
        else if(beaconColor == BeaconColor.BLUE)
            telemetry.log().add("Beacon color is: BLUE");
        else if(beaconColor == null)
            telemetry.log().add("Beacon color is: NULL");

        AutoStateMachineBuilder autoStateMachineBuilder = new AutoStateMachineBuilder(S.WAIT);

        autoStateMachineBuilder.addToWhiteLine(S.TO_WHITE_LINE, S.FOLLOW_lINE, leftMotor, rightMotor, lightSensor);
        autoStateMachineBuilder.addLineFollow(telemetry, S.FOLLOW_lINE, S.PRESS_BEACON, leftMotor, rightMotor, lightSensor, rangeSensor, beaconColor);
        autoStateMachineBuilder.addPressBeacon(telemetry, S.PRESS_BEACON, S.WAIT, leftMotor, rightMotor, leftColorSensor, rightColorSensor, beaconColor);
        autoStateMachineBuilder.addWait(S.WAIT, S._2ND_TO_WHITE_LINE, 3000);
        autoStateMachineBuilder.addToWhiteLine(S._2ND_TO_WHITE_LINE, S._2ND_FOLLOW_lINE, leftMotor, rightMotor, lightSensor);
        autoStateMachineBuilder.addLineFollow(telemetry, S._2ND_FOLLOW_lINE, S._2ND_PRESS_BEACON, leftMotor, rightMotor, lightSensor, rangeSensor, beaconColor);
        autoStateMachineBuilder.addPressBeacon(telemetry, S._2ND_PRESS_BEACON, S.STOP, leftMotor, rightMotor, leftColorSensor, rightColorSensor, beaconColor);
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
        telemetry.update();

        stateMachine.act();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
