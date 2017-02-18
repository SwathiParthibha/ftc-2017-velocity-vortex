package org.firstinspires.ftc.teamcode.Shashank.statemachine.states;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.StateName;

/**
 * Created by spmeg on 1/21/2017.
 */

public class TurnState extends BasicAbstractState {
    private DcMotor leftMotor, rightMotor = null;

    private StateName stateName = null;
    private StateName nextStateName = null;

    private BNO055IMU imu = null;

    private boolean hasInitialized = false;

    Orientation angles;
    double angleZ;
    private int turnAngle;
    TurnDirection direction;
    private double angDiff;

    double TURN_POWER_1 = .2;
    double TURN_POWER_2 = .05;

    public TurnState(StateName stateName, StateName nextStateName, DcMotor leftMotor, DcMotor rightMotor, BNO055IMU imu, int turnAngle, TurnDirection turnDirection) {
        this.stateName = stateName;
        this.nextStateName = nextStateName;
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.imu = imu;
        this.turnAngle = turnAngle;
        this.direction = turnDirection;
    }

    @Override
    public void init() {
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        angleZ = getIMUheading();

        angDiff = turnAngle-angleZ; //positive: turn left
        angDiff = (angDiff + 180) % 360 - 180; //changes to number between -180 and 180

        if (angDiff > 0)
            direction = TurnDirection.LEFT;
        else if (angDiff < 0)
            direction = TurnDirection.RIGHT;
    }

    @Override
    public StateName act() {
        if(!hasInitialized) {
            init();
            hasInitialized = true;
        }

        angleZ = getIMUheading();
        angDiff = turnAngle-angleZ;
        angDiff = (angDiff + 180) % 360 - 180;

        if(!isDone()) {
            if(direction == TurnDirection.LEFT){
                if (Math.abs(angDiff) < 90) {
                    leftMotor.setPower(TURN_POWER_1);
                    rightMotor.setPower(-TURN_POWER_1);
                } else if (Math.abs(angDiff) < 45) {
                    leftMotor.setPower(TURN_POWER_2);
                    rightMotor.setPower(-TURN_POWER_2);
                }

                return stateName;
            } else if(direction == TurnDirection.RIGHT){
                if (Math.abs(angDiff) < 90) {
                    leftMotor.setPower(-TURN_POWER_1);
                    rightMotor.setPower(TURN_POWER_1);
                } else if (Math.abs(angDiff) < 45) {
                    leftMotor.setPower(-TURN_POWER_2);
                    rightMotor.setPower(TURN_POWER_2);
                }

                return stateName;
            } else {
                return stateName;
            }
        } else {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            return nextStateName;
        }
    }

    double getIMUheading() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    @Override
    public boolean isDone() {
        if (direction == TurnDirection.LEFT)
            return angDiff < 0;
        else if (direction == TurnDirection.RIGHT)
            return angDiff > 0;
        else return true;
    }

    public enum TurnDirection {
        LEFT,
        RIGHT
    }

    @Override
    public StateName getNextStateName() {
        return nextStateName;
    }
}
