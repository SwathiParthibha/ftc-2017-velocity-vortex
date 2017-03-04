package org.firstinspires.ftc.teamcode.Shashank.statemachine.states;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.StateName;

/**
 * Created by spmeg on 2/25/2017.
 */

public class ShootState extends BasicAbstractState {
    private DcMotor scooper;
    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor sweeper;
    private Servo leftArm;
    private Servo rightArm;
    private final double LEFT_IN_VAL = 0.56;
    private final double RIGHT_IN_VAL = 0.34;
    private final double LEFT_OUT_VAL = 0.12;
    private final double RIGHT_OUT_VAL = 0.76;
    private StateName stateName, nextStateName;

    private ElapsedTime elapsedTime = new ElapsedTime();

    public ShootState(StateName stateName, StateName nextStateName, DcMotor scooper, DcMotor shooter1, DcMotor shooter2, DcMotor sweeper, Servo leftArm, Servo rightArm) {
        this.scooper = scooper;
        this.shooter1 = shooter1;
        this.shooter2 = shooter2;
        this.sweeper = sweeper;
        this.leftArm = leftArm;
        this.rightArm = rightArm;
        this.stateName = stateName;
        this.nextStateName = nextStateName;
    }

    @Override
    public void init() {
        leftArm.setPosition((LEFT_IN_VAL + LEFT_OUT_VAL)/2);
        rightArm.setPosition((RIGHT_IN_VAL + RIGHT_OUT_VAL)/2);
        elapsedTime.reset();
    }

    @Override
    public StateName act() {
        if(isDone())
            return getNextStateName();
        shooter1.setPower(0.5);
        shooter1.setPower(0.5);

        if(elapsedTime.seconds() > 2){
            shooter1.setPower(0);
            shooter1.setPower(0);
            leftArm.setPosition(LEFT_OUT_VAL);
            rightArm.setPosition(RIGHT_OUT_VAL);
        }
        if(elapsedTime.seconds() > 0.5){
            leftArm.setPosition(LEFT_IN_VAL);
            rightArm.setPosition(RIGHT_IN_VAL);
        }
        return stateName;
    }

    @Override
    public boolean isDone() {
        return elapsedTime.seconds() > 4;
    }

    @Override
    public StateName getNextStateName() {
        return nextStateName
                ;
    }
}
