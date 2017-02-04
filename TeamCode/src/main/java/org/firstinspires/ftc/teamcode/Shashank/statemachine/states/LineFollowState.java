package org.firstinspires.ftc.teamcode.Shashank.statemachine.states;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.BeaconColor;
import org.firstinspires.ftc.teamcode.Shashank.utils.RangeSensorRunnable;
import org.firstinspires.ftc.teamcode.Shashank.utils.ThreadSharedObject;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.StateName;

/**
 * Created by spmeg on 1/21/2017.
 */

public class LineFollowState extends BasicAbstractState {
    private DcMotor leftMotor, rightMotor = null;

    private StateName stateName = null;
    private StateName nextStateName = null;

    private LightSensor lightSensor = null;
    private I2cDeviceSynchImpl rangeSensor = null;

    private Telemetry telemetry = null;

    private boolean hasInitialized = false;

    private RangeSensorRunnable rangeSensorRunnable = null;

    private ThreadSharedObject threadSharedObject = new ThreadSharedObject();

    ElapsedTime runtime = new ElapsedTime();

    private BeaconColor color = null;

    public LineFollowState(Telemetry telemetry, StateName stateName, StateName nextStateName, DcMotor leftMotor, DcMotor rightMotor, LightSensor lightSensor, I2cDeviceSynchImpl rangeSensor, BeaconColor color) {
        this.stateName = stateName;
        this.nextStateName = nextStateName;
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.lightSensor = lightSensor;
        this.telemetry = telemetry;
        this.rangeSensor = rangeSensor;
        this.color = color;
    }

    @Override
    public void init() {
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lightSensor.enableLed(true);

        rangeSensorRunnable = new RangeSensorRunnable(this.telemetry, runtime, rangeSensor, threadSharedObject);
        new Thread(rangeSensorRunnable).start();

        telemetry.log().add("In "+ stateName.name()+" and finished init" );
    }

    @Override
    public StateName act() {
        if(!hasInitialized) {
            init();
            hasInitialized = true;
        }

        telemetry.log().add("range sensor values"+ threadSharedObject.getInteger(rangeSensorRunnable.getULTRASONIC_KEY())+ " and the bool: " + isDone()+ " Light sensor: "+ lightSensor.getLightDetected());
        telemetry.update();

        if(!isDone()) {
            if(color == BeaconColor.BLUE){
                if (lightSensor.getLightDetected() > 0.3) {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0.2);
                } else {
                    leftMotor.setPower(0.2);
                    rightMotor.setPower(0);
                }
                return stateName;
            } else if(color == BeaconColor.RED){
                if (lightSensor.getLightDetected() > 0.3) {
                    leftMotor.setPower(0.2);
                    rightMotor.setPower(0);
                } else {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0.2);
                }
                return stateName;
            } else {
                return stateName;
            }
        } else {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            rangeSensorRunnable.requestStop();
            return nextStateName;
        }
    }

    @Override
    public boolean isDone() {
        return threadSharedObject.getInteger(rangeSensorRunnable.getULTRASONIC_KEY()) < 11 && threadSharedObject.getInteger(rangeSensorRunnable.getULTRASONIC_KEY()) > 0;
    }

    @Override
    public StateName getNextStateName() {
        return nextStateName;
    }
}
