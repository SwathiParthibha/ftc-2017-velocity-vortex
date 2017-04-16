package org.firstinspires.ftc.teamcode.Shashank.statemachine;

import org.firstinspires.ftc.teamcode.Shashank.statemachine.states.MecanumDriveState;

import ftc.electronvolts.statemachine.EndCondition;
import ftc.electronvolts.statemachine.StateMachineBuilder;
import ftc.electronvolts.statemachine.StateName;
import trclib.TrcDriveBase;

/**
 * Created by spmeg on 4/13/2017.
 */

public class MecanumStateMachineBuilder extends StateMachineBuilder {
    private TrcDriveBase trcDriveBase = null;

    public MecanumStateMachineBuilder(StateName firstStateName, TrcDriveBase driveBase) {
        super(firstStateName);
        trcDriveBase = driveBase;
    }

    public void addDrivePolar(StateName stateName, TrcDriveBase trcDriveBase, double magnitude, double direction, double rotation, StateName nextStateName, EndCondition endCondition){
        add(stateName, new MecanumDriveState(trcDriveBase, endCondition, stateName, nextStateName, magnitude, direction, rotation));
    }

}
