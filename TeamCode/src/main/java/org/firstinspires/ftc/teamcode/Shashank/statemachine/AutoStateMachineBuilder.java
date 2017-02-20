package org.firstinspires.ftc.teamcode.Shashank.statemachine;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.states.TurnState;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.states.TurnStateEncoderDrive;
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

    public void addLineFollow(Telemetry telemetry, StateName stateName, StateName nextStateName, DcMotor leftMotor, DcMotor rightMotor, LightSensor lightSensor, I2cDeviceSynchImpl rangeSensor, AllianceColor color){
        add(stateName, AutoStates.lineFollow(telemetry, stateName, nextStateName, leftMotor, rightMotor, lightSensor, rangeSensor, color));
    }

    public void addPressBeacon(Telemetry telemetry, StateName stateName, StateName nextStateName, DcMotor leftMotor, DcMotor rightMotor, ColorSensor leftColorSensor, ColorSensor rightColorSensor, AllianceColor color){
        telemetry.log().add("in add method");
        telemetry.log().add("left motor name: " + leftColorSensor.getConnectionInfo() + " right motor name: "+ leftColorSensor.getConnectionInfo());
        telemetry.update();
        add(stateName, AutoStates.pressBeacon(telemetry, stateName, nextStateName, leftMotor, rightMotor, leftColorSensor, rightColorSensor, color));
    }

    public void addToWhiteLine(StateName stateName, StateName nextStateName, DcMotor leftMotor, DcMotor rightMotor, LightSensor lightSensor){
        add(stateName, AutoStates.toWhiteLine(stateName, nextStateName, leftMotor, rightMotor, lightSensor));
    }

    public void addPivotToWhiteLine(DcMotor leftMotor, DcMotor rightMotor, LightSensor lightSensor, StateName stateName, StateName nextStateName, AllianceColor beaconColor){
        add(stateName, AutoStates.pivotToWhiteLineState(leftMotor, rightMotor, lightSensor, stateName, nextStateName, beaconColor));
    }

    public void addEncoderDrive(DcMotor leftMotor, DcMotor rightMotor, StateName stateName, StateName nextStateName, double distance){
        add(stateName, AutoStates.encoderDrive(leftMotor, rightMotor, stateName, nextStateName, distance));
    }

    public void addTurn(DcMotor leftMotor, DcMotor rightMotor, StateName stateName, StateName nextStateName, BNO055IMU imu, int turnAngle){
        add(stateName, AutoStates.turn(leftMotor, rightMotor, stateName, nextStateName, imu, turnAngle));
    }

    public void addTurnEncoderDrive(DcMotor leftMotor, DcMotor rightMotor, StateName stateName, StateName nextStateName, int turnAngle, TurnStateEncoderDrive.TurnDirection turnDirection){
        add(stateName, AutoStates.turnEncoderDrive(leftMotor, rightMotor, stateName, nextStateName, turnAngle, turnDirection));
    }

}
