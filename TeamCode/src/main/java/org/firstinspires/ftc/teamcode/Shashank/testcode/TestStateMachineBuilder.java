package org.firstinspires.ftc.teamcode.Shashank.testcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import ftc.electronvolts.statemachine.StateMachineBuilder;
import ftc.electronvolts.statemachine.StateName;

/**
 * Created by spmeg on 1/20/2017.
 */

public class TestStateMachineBuilder extends StateMachineBuilder {
    public TestStateMachineBuilder(StateName firstStateName) {
        super(firstStateName);
    }

    public void addTelem(StateName stateName, StateName nextStateName, Telemetry telemetry) {
        add(stateName, TestStates.telemetry(stateName, nextStateName, telemetry));
    }

    public void addDrive(StateName stateName, StateName nextStateName, DcMotor leftMotor, DcMotor rightMotor) {
        add(stateName, TestStates.drive(stateName, nextStateName, leftMotor, rightMotor));
    }

}
