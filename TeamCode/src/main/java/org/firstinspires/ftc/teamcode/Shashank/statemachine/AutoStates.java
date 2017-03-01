package org.firstinspires.ftc.teamcode.Shashank.statemachine;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.states.EncoderDriveState;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.states.LineFollowState;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.states.PivotToWhiteLineState;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.states.PressBeaconState;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.states.ShootState;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.states.ToWhiteLineState;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.states.TurnState;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.states.TurnStateEncoderDrive;
import org.firstinspires.ftc.teamcode.Shashank.testcode.TestMoveState;
import org.firstinspires.ftc.teamcode.Shashank.testcode.TestState;

import ftc.electronvolts.statemachine.State;
import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.statemachine.States;

/**
 * Created by spmeg on 1/20/2017.
 */

public class AutoStates extends States {

    public static State telemetry(Telemetry telemetry, StateName stateName, final StateName nextStateName){
        telemetry.log().add("Now returning state from method telemetry");
        return new TestState(nextStateName, 2);
    }

    public static State drive(StateName stateName, final StateName nextStateName, DcMotor leftMotor, DcMotor rightMotor){
        return new TestMoveState(stateName, nextStateName, 1, leftMotor, rightMotor);
    }

    public static State lineFollow(Telemetry telemetry, StateName stateName, StateName nextStateName, DcMotor leftMotor, DcMotor rightMotor, LightSensor lightSensor, I2cDeviceSynchImpl rangeSensor, AllianceColor color){
        return new LineFollowState(telemetry, stateName, nextStateName, leftMotor, rightMotor, lightSensor,  rangeSensor, color);
    }

    public static State pressBeacon(Telemetry telemetry, StateName stateName, StateName nextStateName, DcMotor leftMotor, DcMotor rightMotor, ColorSensor leftColorSensor, ColorSensor rightColorSensor, AllianceColor color){
        telemetry.log().add("in autostates pressBeacon method");
        telemetry.log().add("left motor name: " + leftColorSensor.getConnectionInfo() + " right motor name: "+ rightColorSensor.getConnectionInfo());
        telemetry.update();
        return new PressBeaconState(stateName, nextStateName, leftMotor, rightMotor, leftColorSensor, rightColorSensor, telemetry, 100, color);
    }

    public static State toWhiteLine(StateName stateName, StateName nextStateName, DcMotor leftMotor, DcMotor rightMotor, LightSensor lightSensor){
        return new ToWhiteLineState(leftMotor, rightMotor, lightSensor, stateName, nextStateName);
    }

    public static State pivotToWhiteLineState(DcMotor leftMotor, DcMotor rightMotor, LightSensor lightSensor, StateName stateName, StateName nextStateName, AllianceColor beaconColor){
        return new PivotToWhiteLineState(leftMotor, rightMotor, lightSensor, stateName, nextStateName, beaconColor);
    }

    public static State encoderDrive(DcMotor leftMotor, DcMotor rightMotor, StateName stateName, StateName nextStateName, double distance){
        return new EncoderDriveState(distance, leftMotor, rightMotor, stateName, nextStateName);
    }

    public static State turn(DcMotor leftMotor, DcMotor rightMotor, StateName stateName, StateName nextStateName, BNO055IMU imu, int turnAngle){
        return new TurnState(stateName, nextStateName, leftMotor, rightMotor, imu, turnAngle);
    }

    public static State turnEncoderDrive(DcMotor leftMotor, DcMotor rightMotor, StateName stateName, StateName nextStateName, int turnAngle, TurnStateEncoderDrive.TurnDirection turnDirection){
        return new TurnStateEncoderDrive(stateName, nextStateName, leftMotor, rightMotor, turnAngle, turnDirection);
    }

    public static State shootWithoutPID(StateName stateName, StateName nextStateName, DcMotor scooper, DcMotor shooter1, DcMotor shooter2, DcMotor sweeper, Servo leftArm, Servo rightArm){
        return new ShootState(stateName, nextStateName, scooper, shooter1, shooter2, sweeper, leftArm, rightArm);
    }

}
