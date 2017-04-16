package org.firstinspires.ftc.teamcode.Shashank.statemachine.states;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.EndCondition;
import ftc.electronvolts.statemachine.StateName;
import trclib.TrcDriveBase;

/**
 * Created by spmeg on 4/13/2017.
 */

public class MecanumDriveState extends BasicAbstractState{
    private TrcDriveBase trcDriveBase = null;
    private EndCondition endCondition = null;
    private StateName nextStateName, stateName;
    private double magnitude = 0.0;
    private double direction = 0.0;
    private double rotation = 0.0;

    public MecanumDriveState(TrcDriveBase trcDriveBase, EndCondition endCondition) {
        this.trcDriveBase = trcDriveBase;
        this.endCondition = endCondition;
    }

    public MecanumDriveState(TrcDriveBase trcDriveBase, EndCondition endCondition, StateName stateName, StateName nextStateName) {
        this(trcDriveBase, endCondition);
        this.nextStateName = nextStateName;
        this.stateName = stateName;
    }

    public MecanumDriveState(TrcDriveBase trcDriveBase, EndCondition endCondition, StateName stateName, StateName nextStateName, double magnitude, double direction, double rotation) {
        this(trcDriveBase, endCondition, stateName, nextStateName);
        this.magnitude = magnitude;
        this.direction = direction;
        this.rotation = rotation;
    }

    @Override
    public void init() {
        //do INIT stuff here
        endCondition.init();
    }

    @Override
    public StateName act() {
        trcDriveBase.mecanumDrive_Polar(magnitude, direction, rotation);
        if(isDone()){
            trcDriveBase.mecanumDrive_Polar(0.0, direction, 0.0);
            return nextStateName;
        } else {
            return stateName;
        }
    }

    @Override
    public boolean isDone() {
        return endCondition.isDone();
    }

    @Override
    public StateName getNextStateName() {
        return null;
    }
}
