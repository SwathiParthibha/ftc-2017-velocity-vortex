package org.firstinspires.ftc.teamcode.Shashank.statemachine.states;

import android.os.AsyncTask;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.StateName;

/**
 * Created by spmeg on 1/21/2017.
 */

public class TurnStateEncoderDrive extends BasicAbstractState {
    private DcMotor leftMotor, rightMotor = null;

    private StateName stateName = null;
    private StateName nextStateName = null;

    private BNO055IMU imu = null;

    private boolean hasInitialized = false;

    private boolean stopThread = false;

    Orientation angles;
    double angleZ;
    private int turnAngle;
    TurnDirection direction;
    private double angDiff;

        double TURN_POWER_1 = .2;
        double TURN_POWER_2 = .05;

        public TurnStateEncoderDrive(StateName stateName, StateName nextStateName, DcMotor leftMotor, DcMotor rightMotor, BNO055IMU imu, int turnAngle) {
            this.stateName = stateName;
            this.nextStateName = nextStateName;
            this.leftMotor = leftMotor;
            this.rightMotor = rightMotor;
            this.imu = imu;
            this.turnAngle = turnAngle;
    }

    @Override
    public void init() {
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        angleZ = getIMUheading();

        angDiff = turnAngle-angleZ; //positive: turn left

        AsyncTask.execute(new Runnable() {
            @Override
            public void run() {
                while (!stopThread){
                    angleZ = getIMUheading();
                }
            }
        });

        DbgLog.msg("FNINSHED INIT");
    }

    @Override
    public StateName act() {
        if(!hasInitialized) {
            init();
            hasInitialized = true;
        }

        DbgLog.msg("angDiff is " + angDiff);
        DbgLog.msg("turn angle is " + turnAngle);
        DbgLog.msg("angleZ angle is " + angleZ);

        //angleZ = getIMUheading();
        angDiff = turnAngle-angleZ;

        if(!isDone()) {
            if(angDiff < 0){
                leftMotor.setPower(TURN_POWER_1);
                rightMotor.setPower(-TURN_POWER_1);
                return stateName;
            } else if(angDiff > 0){
                leftMotor.setPower(-TURN_POWER_1);
                rightMotor.setPower(TURN_POWER_1);
                return stateName;
            } else {
                return stateName;
            }
        } else {
            DbgLog.msg("IS DONE");
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            return nextStateName;
        }
    }

    double getIMUheading() {
        return Math.abs(imu.getAngularOrientation().firstAngle);
    }

    @Override
    public boolean isDone() {
        return Math.abs(angDiff) < 3;
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
