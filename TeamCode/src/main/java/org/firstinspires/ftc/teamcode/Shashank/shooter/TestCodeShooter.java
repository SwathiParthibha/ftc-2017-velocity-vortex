package org.firstinspires.ftc.teamcode.Shashank.shooter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import hallib.HalDashboard;
import trclib.TrcKalmanFilter;

/**
 * Created by spmeg on 4/20/2017.
 */
@TeleOp(name = "TestCodeShooter", group = "Test")
public class TestCodeShooter extends OpMode {
    private DcMotor leftShooter, rightShooter;
    private double MOTOR_POWER_INCREMENT = 0.001;
    private double SHOOTER_POWER = 0.5;
    private VoltageSensor voltageSensor = null;
    private DcMotor scooper;
    private Servo leftArm;
    private Servo rightArm;
    private double k = 8.708;

    private final double MAX_POWER = 1.0;
    private final double MIN_POWER = -1.0;
    private final double ZERO_POWER = 0.0;

    private final double LEFT_IN_VAL = 0.56;
    private final double RIGHT_IN_VAL = 0.37;
    private final double LEFT_OUT_VAL = 0.12;
    private final double RIGHT_OUT_VAL = 0.77;
    private final double LEFT_ARMED_VAL = 0.25;
    private final double RIGHT_ARMED_VAL = 0.65;

    private double calculatedValue = 0.0;

    private double LEFT_POWER = 0.65;
    private double RIGHT_POWER = 0.65;

    private volatile double LEFT_RPM = 0;
    private volatile double RIGHT_RPM = 0;

    private double MAX_RPM_DIFF = 1.5;
    private int LOW_TARGET_RPM = 71;
    private int HIGH_TARGET_RPM = 77;
    private int TARGET_RPM = LOW_TARGET_RPM;

    private Runnable servoRunnable = null;

    private ScheduledExecutorService scheduledExecutorService = Executors.newScheduledThreadPool(110);

    @Override
    public void init() {

        leftShooter = this.hardwareMap.dcMotor.get("shooter1");
        rightShooter = this.hardwareMap.dcMotor.get("shooter2");

        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        scooper = this.hardwareMap.dcMotor.get("scooper");
        leftArm = this.hardwareMap.servo.get("leftservo");
        rightArm = this.hardwareMap.servo.get("rightservo");

        voltageSensor = this.hardwareMap.voltageSensor.get("sweepec");

        for (int i = 0; i < 100; i++) {
            scheduledExecutorService.scheduleAtFixedRate(new Runnable() {
                private int prevLeft, prevRight;

                @Override
                public void run() {
                    LEFT_RPM = (leftShooter.getCurrentPosition() - prevLeft)/28.0;
                    RIGHT_RPM = (rightShooter.getCurrentPosition() - prevRight)/28.0;
                    prevLeft = leftShooter.getCurrentPosition();
                    prevRight = rightShooter.getCurrentPosition();
                }
            }, i * 10, 1000, TimeUnit.MILLISECONDS);
        }

        scheduledExecutorService.scheduleAtFixedRate(new Runnable() {
            @Override
            public void run() {
                if (gamepad2.b) {
                    if(LEFT_RPM - TARGET_RPM < MAX_RPM_DIFF){
                        if(LEFT_RPM - TARGET_RPM < MAX_RPM_DIFF*2)
                            LEFT_POWER += MOTOR_POWER_INCREMENT*3;
                        else
                            LEFT_POWER += MOTOR_POWER_INCREMENT;
                    } else if (LEFT_RPM - TARGET_RPM > MAX_RPM_DIFF){
                        if(LEFT_RPM - TARGET_RPM > MAX_RPM_DIFF*2)
                            LEFT_POWER -= MOTOR_POWER_INCREMENT*3;
                        else
                            LEFT_POWER -= MOTOR_POWER_INCREMENT;
                    }

                    if(RIGHT_RPM - TARGET_RPM < MAX_RPM_DIFF){
                        if(RIGHT_RPM - TARGET_RPM < MAX_RPM_DIFF*2)
                            RIGHT_POWER += MOTOR_POWER_INCREMENT*3;
                        else
                            RIGHT_POWER += MOTOR_POWER_INCREMENT;
                    } else if (RIGHT_RPM - TARGET_RPM > MAX_RPM_DIFF){
                        if(RIGHT_RPM - TARGET_RPM > MAX_RPM_DIFF*2)
                            RIGHT_POWER -= MOTOR_POWER_INCREMENT*3;
                        else
                            RIGHT_POWER -= MOTOR_POWER_INCREMENT;
                    }

                    LEFT_POWER = Range.clip(LEFT_POWER, 0.38, 1);
                    RIGHT_POWER = Range.clip(RIGHT_POWER, 0.38, 1);
                }
            }
        }, 0, 50, TimeUnit.MILLISECONDS);

        servoRunnable = new Runnable() {
            @Override
            public void run() {
                long startTime = System.currentTimeMillis();
                while (System.currentTimeMillis() - startTime < 600){
                    leftArm.setPosition(LEFT_ARMED_VAL);
                    rightArm.setPosition(RIGHT_ARMED_VAL);
                }
                leftArm.setPosition(LEFT_IN_VAL);
                rightArm.setPosition(RIGHT_IN_VAL);
            }
        };
    }

    @Override
    public void loop() {
        if (gamepad2.left_trigger > 0) {
            scooper.setPower(MAX_POWER);

            leftArm.setPosition(LEFT_OUT_VAL);
            rightArm.setPosition(RIGHT_OUT_VAL);
        } else if (gamepad2.left_bumper) {
            scooper.setPower(MIN_POWER);
        } else {
            scooper.setPower(ZERO_POWER);
        }

        if (gamepad2.right_bumper) {
            leftArm.setPosition(LEFT_ARMED_VAL);
            rightArm.setPosition(RIGHT_ARMED_VAL);
        }

        if (gamepad2.dpad_left) {
            leftArm.setPosition(LEFT_OUT_VAL);
            rightArm.setPosition(RIGHT_OUT_VAL);
        } else if (gamepad2.dpad_right) {
            //scheduledExecutorService.schedule(servoRunnable, 0, TimeUnit.MILLISECONDS);
            long startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < 600){
                leftArm.setPosition(LEFT_ARMED_VAL);
                rightArm.setPosition(RIGHT_ARMED_VAL);
            }
            leftArm.setPosition(LEFT_IN_VAL);
            rightArm.setPosition(RIGHT_IN_VAL);
        }

        if (gamepad2.dpad_up) {
            SHOOTER_POWER += MOTOR_POWER_INCREMENT;
        } else if (gamepad1.dpad_down){
            SHOOTER_POWER -= MOTOR_POWER_INCREMENT;
        }

        if(gamepad2.a){
            leftShooter.setPower(SHOOTER_POWER);
            rightShooter.setPower(SHOOTER_POWER);
        } else if(gamepad2.b){
            leftShooter.setPower(LEFT_POWER);
            rightShooter.setPower(RIGHT_POWER);
        } else {
            leftShooter.setPower(0);
            rightShooter.setPower(0);
        }

        if(gamepad2.y){
            TARGET_RPM = HIGH_TARGET_RPM;
        }

        if(gamepad2.x){
            TARGET_RPM = LOW_TARGET_RPM;
        }

        if(k/voltageSensor.getVoltage() > 1.0){
            telemetry.addData("WARNING", "voltage too low");
        }

        telemetry.addData("target rpm", TARGET_RPM);
        telemetry.addData("shooter power", SHOOTER_POWER);
        telemetry.addData("left power", LEFT_POWER);
        telemetry.addData("right power", RIGHT_POWER);
        telemetry.addData("left rpm", LEFT_RPM);
        telemetry.addData("right rpm", RIGHT_RPM);
        telemetry.addData("left pos", leftShooter.getCurrentPosition());
        telemetry.addData("right pos", rightShooter.getCurrentPosition());
        telemetry.update();
    }
}
