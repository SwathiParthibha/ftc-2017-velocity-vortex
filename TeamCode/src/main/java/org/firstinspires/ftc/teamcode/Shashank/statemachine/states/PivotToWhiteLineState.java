package org.firstinspires.ftc.teamcode.Shashank.statemachine.states;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.firstinspires.ftc.teamcode.Shashank.statemachine.BeaconColor;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.StateName;

/**
 * Created by spmeg on 2/3/2017.
 */

public class PivotToWhiteLineState extends BasicAbstractState {
    private DcMotor leftMotor, rightMotor;

    private LightSensor lightSensor;

    private StateName stateName, nextStateName;

    private static final double WHITE_LINE_THRESHOLD = 0.3;

    private BeaconColor beaconColor;

    private boolean hasInitialized = false;

    public PivotToWhiteLineState(DcMotor leftMotor, DcMotor rightMotor, LightSensor lightSensor, StateName stateName, StateName nextStateName, BeaconColor beaconColor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.lightSensor = lightSensor;
        this.stateName = stateName;
        this.nextStateName = nextStateName;
        this.beaconColor = beaconColor;
    }

    @Override
    public StateName act() {
        if(!hasInitialized) {
            init();
            hasInitialized = true;
        }

        if(isDone()) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            return getNextStateName();
        } else {
            if(beaconColor == BeaconColor.BLUE){
                leftMotor.setPower(0.4);
                rightMotor.setPower(-0.4);
            } else {
                leftMotor.setPower(-0.4);
                rightMotor.setPower(0.4);
            }

            return stateName;
        }
    }

    @Override
    public void init() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public boolean isDone() {
        return lightSensor.getLightDetected() > WHITE_LINE_THRESHOLD;
    }

    @Override
    public StateName getNextStateName() {
        return nextStateName;
    }
}
