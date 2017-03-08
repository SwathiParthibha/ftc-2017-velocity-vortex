package org.firstinspires.ftc.teamcode.Shashank.statemachine.states;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Sam.shooter.MotorFactory;
import org.firstinspires.ftc.teamcode.Sam.shooter.RPMThreadMilliseconds;
import org.firstinspires.ftc.teamcode.Sam.shooter.beans.ShooterMotor;
import org.firstinspires.ftc.teamcode.Sam.shooter.power.PowerManager;
import org.firstinspires.ftc.teamcode.Sam.shooter.util.Constants;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

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
    private PowerManager leftShooterPowerMgr;
    private PowerManager rightShooterPowerMgr;
    private ScheduledExecutorService scheduledThreadPool = Executors.newScheduledThreadPool(2);
    private ShooterMotor leftShooter, rightShooter;
    private ElapsedTime elapsedTime = new ElapsedTime();
    private boolean hasInit = false;
    private long startingTime = 0;
    private static final long NANO_PER_SEC = 1000000000;

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

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorFactory motorFactory = MotorFactory.getInstance();
        leftShooter = new ShooterMotor();
        leftShooter.setName(Constants.MOTORNAME.LEFT_SHOOTER);
        motorFactory.addMotor(leftShooter);

        rightShooter = new ShooterMotor();
        rightShooter.setName(Constants.MOTORNAME.RIGHT_SHOOTER);
        motorFactory.addMotor(rightShooter);

        leftShooterPowerMgr = new PowerManager(Constants.MOTORNAME.LEFT_SHOOTER, shooter1);
        rightShooterPowerMgr = new PowerManager(Constants.MOTORNAME.RIGHT_SHOOTER, shooter2);

        scheduledThreadPool.scheduleAtFixedRate(new RPMThreadMilliseconds(shooter1, Constants.MOTORNAME.LEFT_SHOOTER), 0L, Constants.DELTA_TIME, TimeUnit.MILLISECONDS);
        scheduledThreadPool.scheduleAtFixedRate(new RPMThreadMilliseconds(shooter2, Constants.MOTORNAME.RIGHT_SHOOTER), 0L, Constants.DELTA_TIME, TimeUnit.MILLISECONDS);

        Constants.REQUESTED_ETPS = 1650;//1590;//1750 good for close shots
        Constants.DEFAULT_POWER = 0.45;//0.455;//0.42
        startingTime = System.nanoTime();
    }

    @Override
    public StateName act() {
        if(!hasInit)
            init();

        if(isDone())
            return getNextStateName();

        leftShooterPowerMgr.regulatePower();
        rightShooterPowerMgr.regulatePower();

        if(getTimeDifference() > 2*NANO_PER_SEC){
            shooter1.setPower(0);
            shooter1.setPower(0);
            leftArm.setPosition(LEFT_OUT_VAL);
            rightArm.setPosition(RIGHT_OUT_VAL);
        }
        if(getTimeDifference() > 0.9*NANO_PER_SEC){
            leftArm.setPosition(LEFT_IN_VAL);
            rightArm.setPosition(RIGHT_IN_VAL);
        }
        return stateName;
    }

    private long getTimeDifference(){
        return System.nanoTime() - startingTime;
    }

    @Override
    public boolean isDone() {
        return elapsedTime.seconds() > 4;
    }

    @Override
    public StateName getNextStateName() {
        return nextStateName;
    }
}
