package org.firstinspires.ftc.teamcode.Shashank.testcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.StateName;

/**
 * Created by spmeg on 1/20/2017.
 */

public class TestMoveState extends BasicAbstractState {
    private StateName nextStateName;
    private StateName stateName;
    private ElapsedTime time = new ElapsedTime();
    private int timeout=0;
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private boolean hasInitialized = false;

    public TestMoveState(StateName stateName, StateName nextStateName, int timeout, DcMotor leftMotor, DcMotor rightMotor) {
        this.stateName = stateName;
        this.nextStateName = nextStateName;
        this.timeout = timeout;
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
    }

    @Override
    public StateName act() {
        if (!hasInitialized){
            init();
            hasInitialized = true;
        }

        leftMotor.setPower(0.2);
        rightMotor.setPower(0.2);

        if (isDone()) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            return getNextStateName();
        }
        return stateName;
    }

    @Override
    public void init() {
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        time.reset();
    }

    @Override
    public boolean isDone() {
        return time.seconds() > timeout;
    }

    @Override
    public StateName getNextStateName() {
        return nextStateName;
    }
}
