package org.firstinspires.ftc.teamcode.Shashank.statemachine.states;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.BeaconColor;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.StateName;

/**
 * Created by spmeg on 1/21/2017.
 */

public class PressBeaconState extends BasicAbstractState {
    private DcMotor leftMotor, rightMotor = null;

    private StateName stateName = null;
    private StateName nextStateName = null;

    private ColorSensor leftColorSensor, rightColorSensor;

    private Telemetry telemetry = null;

    private boolean hasInitialized = false;

    private ElapsedTime elapsedTime = null;

    private int timeout = 0;

    private BeaconColor beaconColor = null;

    public PressBeaconState(StateName stateName, StateName nextStateName, DcMotor leftMotor, DcMotor rightMotor, ColorSensor leftColorSensor, ColorSensor rightColorSensor, Telemetry telemetry, int timeout, BeaconColor color) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.stateName = stateName;
        this.nextStateName = nextStateName;
        this.leftColorSensor = leftColorSensor;
        this.rightColorSensor = rightColorSensor;
        this.telemetry = telemetry;
        this.timeout = timeout;
        this.beaconColor = color;
    }

    @Override
    public void init() {
        elapsedTime = new ElapsedTime();
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.log().add("Finished init");
    }

    @Override
    public StateName act() {
        if(!hasInitialized) {
            init();
            hasInitialized = true;
            telemetry.log().add("called init");
            telemetry.update();
        }

        telemetry.log().add("isSame(): " + isSameColor());
        telemetry.log().add("isDone(): " + isDone());
        telemetry.log().add("timeout: " + (elapsedTime.seconds() > timeout));
        telemetry.log().add("timeout value: " + timeout);
        telemetry.update();

        if(isDone()) {
            telemetry.log().add("it is done");
            telemetry.update();
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            return nextStateName;
        }
        if(beaconColor == BeaconColor.RED){
            //if red do this
            telemetry.log().add("in RED if");
            telemetry.update();
            if(leftColorSensor.red() > leftColorSensor.blue()
                    && rightColorSensor.red() < rightColorSensor.blue()){
                leftMotor.setPower(0.2);
                rightMotor.setPower(0);
                telemetry.log().add("turn left");
                telemetry.update();
            } else {
                leftMotor.setPower(0);
                rightMotor.setPower(0.2);
                telemetry.log().add("turn right");
                telemetry.update();
            }
        } else if(beaconColor == BeaconColor.BLUE){
            //if blue do this
            telemetry.log().add("in BLUE if");
            if(leftColorSensor.red() < leftColorSensor.blue()
                    && rightColorSensor.red() > rightColorSensor.blue()){
                leftMotor.setPower(0.2);
                rightMotor.setPower(0);
                telemetry.log().add("turn left");
                telemetry.update();
            } else {
                leftMotor.setPower(0);
                rightMotor.setPower(0.2);
                telemetry.log().add("turn right");
                telemetry.update();
            }
        } else {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }

        return stateName;
    }

    @Override
    public boolean isDone() {
        return elapsedTime.seconds() > timeout || isSameColor();
    }

    private boolean isSameColor() {
        if(leftColorSensor.red() > leftColorSensor.blue() && rightColorSensor.red() > rightColorSensor.blue())
                return true;
        else if(leftColorSensor.red() < leftColorSensor.blue() && rightColorSensor.red() < rightColorSensor.blue())
                return true;
        else
            return false;

    }

    @Override
    public StateName getNextStateName() {
        return nextStateName;
    }
}
