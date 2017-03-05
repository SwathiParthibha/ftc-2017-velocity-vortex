package org.firstinspires.ftc.teamcode.Shashank.statemachine.states;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import ftc.electronvolts.statemachine.BasicAbstractState;
import ftc.electronvolts.statemachine.StateName;

/**
 * Created by spmeg on 1/24/2017.
 */

public class ToWhiteLineState extends BasicAbstractState {
    private DcMotor leftMotor, rightMotor;

    private LightSensor lightSensor;

    private int timeout;

    private ElapsedTime runTime;

    private StateName stateName, nextStateName;

    private static final double WHITE_LINE_THRESHOLD = 0.2;

    private boolean hasInitialized = false;
    private ExecutorService executorService = Executors.newSingleThreadExecutor();
    private double lightDetected = 0.0;

    public ToWhiteLineState(DcMotor leftMotor, DcMotor rightMotor, LightSensor lightSensor, StateName stateName, StateName nextStateName) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.lightSensor = lightSensor;
        this.stateName = stateName;
        this.nextStateName = nextStateName;
    }

    @Override
    public void init() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public StateName act() {
        if(!hasInitialized){
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

        if (!isDone()){
            leftMotor.setPower(0.2);
            rightMotor.setPower(0.2);
            return stateName;
        } else {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            executorService.shutdown();
            return nextStateName;
        }
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
