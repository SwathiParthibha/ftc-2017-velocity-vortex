package org.firstinspires.ftc.teamcode.Shashank.statemachine.states;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.firstinspires.ftc.teamcode.Shashank.statemachine.AllianceColor;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.StateName;

/**
 * Created by spmeg on 2/3/2017.
 */

public class PivotToWhiteLineState extends BasicAbstractState {
    private DcMotor leftMotor, rightMotor;

    private LightSensor lightSensor;

    private StateName stateName, nextStateName;

    private static final double WHITE_LINE_THRESHOLD = 0.188;

    private AllianceColor beaconColor;

    private boolean hasInitialized = false;

    private static final double MOTOR_POWER = 0.2;
    private double lightDetected = 0.0;
    private ExecutorService executorService = Executors.newSingleThreadExecutor();

    public PivotToWhiteLineState(DcMotor leftMotor, DcMotor rightMotor, LightSensor lightSensor, StateName stateName, StateName nextStateName, AllianceColor beaconColor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.lightSensor = lightSensor;
        this.stateName = stateName;
        this.nextStateName = nextStateName;
        this.beaconColor = beaconColor;
    }

    @Override
    public StateName act() {
        if(!hasInitialized) {
            init();
            hasInitialized = true;

            executorService.execute(new Runnable() {
                @Override
                public void run() {
                    while (!executorService.isShutdown())
                        lightDetected = lightSensor.getLightDetected();
                }
            });
        }

        if(isDone()) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            executorService.shutdown();
            return getNextStateName();
        } else {
            if(beaconColor == AllianceColor.BLUE){
                leftMotor.setPower(MOTOR_POWER);
                rightMotor.setPower(-MOTOR_POWER);
            } else {
                leftMotor.setPower(-MOTOR_POWER);
                rightMotor.setPower(MOTOR_POWER);
            }

            return stateName;
        }
    }

    @Override
    public void init() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public boolean isDone() {
        return lightDetected > WHITE_LINE_THRESHOLD;
    }

    @Override
    public StateName getNextStateName() {
        return nextStateName;
    }
}
