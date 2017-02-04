package org.firstinspires.ftc.teamcode.Shashank.testcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.StateName;

/**
 * Created by spmeg on 1/20/2017.
 */

public class TestState extends BasicAbstractState {
    private StateName nextStateName;
    private ElapsedTime time = new ElapsedTime();
    private int timeout=0;

    public TestState(StateName nextStateName, int timeout) {
        this.nextStateName = nextStateName;
        this.timeout = timeout;
    }

    @Override
    public void init() {
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
