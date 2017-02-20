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

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 5.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     Circumference           = 18 * Math.PI;
    static final double     Inches_PER_DEGREE       = Circumference / 360;

    Orientation angles;
    private int turnAngle;
    private TurnDirection direction;

    double TURN_POWER_1 = .2;
    double TURN_POWER_2 = .05;

    public TurnStateEncoderDrive(StateName stateName, StateName nextStateName, DcMotor leftMotor, DcMotor rightMotor, int turnAngle, TurnDirection turnDirection) {
        this.stateName = stateName;
        this.nextStateName = nextStateName;
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.turnAngle = turnAngle;
        this.direction = turnDirection;
    }
    @Override
    public void init() {
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int targetPos = (int) (COUNTS_PER_INCH*Inches_PER_DEGREE*turnAngle);

        if(direction == TurnDirection.LEFT){
            leftMotor.setTargetPosition(leftMotor.getCurrentPosition()-targetPos);
            rightMotor.setTargetPosition(leftMotor.getCurrentPosition()+targetPos);
        } else {
            leftMotor.setTargetPosition(leftMotor.getCurrentPosition()+targetPos);
            rightMotor.setTargetPosition(leftMotor.getCurrentPosition()-targetPos);
        }

        DbgLog.msg("FNINSHED INIT");
    }

    @Override
    public StateName act() {
        if(!hasInitialized) {
            init();
            hasInitialized = true;
        }

        DbgLog.msg("turn angle is " + turnAngle);

        if(!isDone()) {
            if(direction == TurnDirection.LEFT){
                leftMotor.setPower(TURN_POWER_1);
                rightMotor.setPower(TURN_POWER_1);
                return stateName;
            } else if(direction == TurnDirection.RIGHT){
                leftMotor.setPower(TURN_POWER_1);
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

    @Override
    public boolean isDone() {
        return leftMotor.getCurrentPosition() > leftMotor.getTargetPosition()
        && rightMotor.getCurrentPosition() > rightMotor.getTargetPosition();
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
