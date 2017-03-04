package org.firstinspires.ftc.teamcode.Shashank.statemachine.states;

import android.os.AsyncTask;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
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
    private static boolean IS_LARGER_THAN_180 = false;
    Orientation angles;
    double angleZ;
    private int turnAngle;
    TurnDirection direction;
    private double angDiff;

    double TURN_POWER_0 = .3;
    double TURN_POWER_1 = .2;
    double TURN_POWER_2 = .1;

    ElapsedTime runtime = new ElapsedTime();

    public TurnState(StateName stateName, StateName nextStateName, DcMotor leftMotor, DcMotor rightMotor, BNO055IMU imu, int turnAngle) {
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
        angDiff = turnAngle-angleZ; //negative: turn left



        //angDiff = (angDiff + 180) % 360 - 180; //changes to number between -180 and 180

        log("angDiff in init is " + angDiff);
        log("turnAngle in init is " + turnAngle);
        log("angleZ in init is " + angleZ);
        log("\n\n");

        log("direction in init is " + direction);
        log("FINISHED INIT");
        log("\n\n");
    }

    @Override
    public StateName act() {
        if(!hasInitialized) {
            init();
            hasInitialized = true;

            AsyncTask.execute(new Runnable() {
                @Override
                public void run() {
                    while (!isDone()){
                        angDiff = getAngleDifference(turnAngle);
                        direction = getDirection();
                    }
                }
            });
        }

        log("angDiff in is " + angDiff);
        log("turnAngle in is " + turnAngle);
        log("angleZ in is " + angleZ);
        log("direction in init is " + direction);
        log("right motor power is " + rightMotor.getPower());
        log("left motor power is " + leftMotor.getPower());
        log("\n\n");

        if(!isDone()) {
            if(direction == TurnDirection.LEFT){
                if (Math.abs(angDiff) >= 90) {
                    leftMotor.setPower(-TURN_POWER_0);
                    rightMotor.setPower(TURN_POWER_0);
                } else if (Math.abs(angDiff) >= 45) {
                    leftMotor.setPower(-TURN_POWER_1);
                    rightMotor.setPower(TURN_POWER_1);
                } else if (Math.abs(angDiff) < 45) {
                    leftMotor.setPower(-TURN_POWER_2);
                    rightMotor.setPower(TURN_POWER_2);
                }
                return stateName;
            } else if(direction == TurnDirection.RIGHT){
                if (Math.abs(angDiff) >= 90) {
                    leftMotor.setPower(TURN_POWER_0);
                    rightMotor.setPower(-TURN_POWER_0);
                } else if (Math.abs(angDiff) >= 45) {
                    leftMotor.setPower(TURN_POWER_1);
                    rightMotor.setPower(-TURN_POWER_1);
                } else if (Math.abs(angDiff) < 45) {
                    leftMotor.setPower(TURN_POWER_2);
                    rightMotor.setPower(-TURN_POWER_2);
                }
                return stateName;
            } else {
                return stateName;
            }
        } else {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            log("FINISHED STATE");
            return nextStateName;
        }
    }

    private TurnDirection getDirection(){
        if (angDiff < 0) {
            if(Math.abs(angDiff) > 180){
                IS_LARGER_THAN_180 = true;
                return TurnDirection.RIGHT;
            }
            return TurnDirection.LEFT;
        }
        else if (angDiff > 0){
            if(Math.abs(angDiff) > 180){
                IS_LARGER_THAN_180 = true;
                return TurnDirection.LEFT;
            }
            return TurnDirection.RIGHT;
        }

        return null;
    }

    private double getAngleDifference(int turnAngle){
        double angDiff = turnAngle-Math.abs(imu.getAngularOrientation().firstAngle);
        if(Math.abs(angDiff) > 180){
            IS_LARGER_THAN_180 = true;
        } else {
            IS_LARGER_THAN_180 = false;
        }

        if(IS_LARGER_THAN_180){
            if(angDiff < 0){
                angDiff = (360 - angleZ) + turnAngle;
            } else if(angDiff > 0){
                angDiff = (360 - turnAngle) + angleZ;
            }
        }
        return angDiff;
    }

    double getIMUheading() {
        return Math.abs(imu.getAngularOrientation().firstAngle);
    }

    @Override
    public boolean isDone() {
        if(Math.abs(angDiff) < 3)
            return true;
        else return false;
    }

    enum TurnDirection {
        LEFT,
        RIGHT
    }

    private void log(String str){
        DbgLog.msg("TURN STATE > " + str);
    }

    @Override
    public StateName getNextStateName() {
        return nextStateName;
    }
}
