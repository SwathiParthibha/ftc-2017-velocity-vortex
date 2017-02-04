package org.firstinspires.ftc.teamcode.Shashank.statemachine;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Shashank.testcode.TestStates;

import ftc.electronvolts.statemachine.StateMachineBuilder;
import ftc.electronvolts.statemachine.StateName;

/**
 * Created by spmeg on 1/20/2017.
 */

public class AutoStateMachineBuilder extends StateMachineBuilder {
    public AutoStateMachineBuilder(StateName firstStateName) {
        super(firstStateName);
    }

    public void addTelem(StateName stateName, StateName nextStateName, Telemetry telemetry) {
        add(stateName, TestStates.telemetry(stateName, nextStateName, telemetry));
    }

    public void addDrive(StateName stateName, StateName nextStateName, DcMotor leftMotor, DcMotor rightMotor) {
        add(stateName, TestStates.drive(stateName, nextStateName, leftMotor, rightMotor));
    }

    public void addLineFollow(Telemetry telemetry, StateName stateName, StateName nextStateName, DcMotor leftMotor, DcMotor rightMotor, LightSensor lightSensor, I2cDeviceSynchImpl rangeSensor, BeaconColor color){
        add(stateName, AutoStates.lineFollow(telemetry, stateName, nextStateName, leftMotor, rightMotor, lightSensor, rangeSensor, color));
    }

    public void addPressBeacon(Telemetry telemetry, StateName stateName, StateName nextStateName, DcMotor leftMotor, DcMotor rightMotor, ColorSensor leftColorSensor, ColorSensor rightColorSensor, BeaconColor color){
        add(stateName, AutoStates.pressBeacon(telemetry, stateName, nextStateName, leftMotor, rightMotor, leftColorSensor, rightColorSensor, color));
    }

    public void addToWhiteLine(StateName stateName, StateName nextStateName, DcMotor leftMotor, DcMotor rightMotor, LightSensor lightSensor){
        add(stateName, AutoStates.toWhiteLine(stateName, nextStateName, leftMotor, rightMotor, lightSensor));
    }

}
