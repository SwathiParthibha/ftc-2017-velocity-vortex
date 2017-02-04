package org.firstinspires.ftc.teamcode.Shashank.testcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.statemachine.States;

/**
 * Created by spmeg on 1/20/2017.
 */

public class TestStates extends States {
    public static State telemetry(StateName stateName, final StateName nextStateName, Telemetry telemetry){
        telemetry.log().add("Now returning state from method telemetry");
        return new TestState(nextStateName, 2);
    }

    public static State drive(StateName stateName, final StateName nextStateName, DcMotor leftMotor, DcMotor rightMotor){
        return new TestMoveState(stateName, nextStateName, 1, leftMotor, rightMotor);
    }

}
