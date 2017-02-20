package org.firstinspires.ftc.teamcode.Shashank.statemachine.states;

import com.qualcomm.robotcore.hardware.DcMotor;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.StateName;

/**
 * Created by spmeg on 2/11/2017.
 */

public class EncoderDriveState extends BasicAbstractState {

    private boolean hasInitialized = false;
    private double distanceInch = 0;

    private DcMotor leftMotor, rightMotor;

    private StateName stateName, nextStateName;

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 5.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE_SPEED             = 0.3;

    public EncoderDriveState(double distanceInch, DcMotor leftMotor, DcMotor rightMotor, StateName stateName, StateName nextStateName) {
        this.distanceInch = distanceInch;
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.stateName = stateName;
        this.nextStateName = nextStateName;
    }

    @Override
    public void init() {

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setTargetPosition((int) (distanceInch * COUNTS_PER_INCH) + leftMotor.getCurrentPosition());
        rightMotor.setTargetPosition((int) (distanceInch * COUNTS_PER_INCH) + rightMotor.getCurrentPosition());

        // Turn On RUN_TO_POSITION
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(DRIVE_SPEED);
        rightMotor.setPower(DRIVE_SPEED);
    }

    @Override
    public StateName act() {
        if(!hasInitialized){
            init();
            hasInitialized = true;
        }

        if(!isDone()){
            return stateName;
        } else {
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return getNextStateName();
        }
    }

    @Override
    public boolean isDone() {
        int leftDiff = Math.abs(leftMotor.getTargetPosition() - leftMotor.getCurrentPosition());
        int rightDiff = Math.abs(rightMotor.getTargetPosition() - rightMotor.getCurrentPosition());
        return leftDiff < 3 && rightDiff < 3;
    }

    @Override
    public StateName getNextStateName() {
        return nextStateName;
    }
}
