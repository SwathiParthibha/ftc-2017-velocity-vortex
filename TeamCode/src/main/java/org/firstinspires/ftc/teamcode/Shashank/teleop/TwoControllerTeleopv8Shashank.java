package org.firstinspires.ftc.teamcode.Shashank.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Created by spmeg on 4/22/2017.
 */
@TeleOp(name = "TwoControllerTeleopv8 Shashank", group = "Teleop")
public class TwoControllerTeleopv8Shashank extends OpMode {
    private DcMotor leftShooter, rightShooter;
    private double MOTOR_POWER_INCREMENT = 0.001;
    private double SHOOTER_POWER = 0.5;
    private VoltageSensor voltageSensor = null;
    private DcMotor scooper;
    private Servo leftArm;
    private Servo rightArm;

    private final double ODS_MAX_VALUE = 0.44;
    private final double ODS_MIN_VALUE = 0.32;
    private final double SWEEPER_IN_POWER = -0.7;
    private final double SWEEPER_OUT_POWER = 0.7;

    private final double MAX_POWER = 1.0;
    private final double MIN_POWER = -1.0;
    private final double ZERO_POWER = 0.0;

    private final double LEFT_IN_VAL = 0.56;
    private final double RIGHT_IN_VAL = 0.37;
    private final double LEFT_OUT_VAL = 0.12;
    private final double RIGHT_OUT_VAL = 0.77;
    private final double LEFT_ARMED_VAL = 0.25;
    private final double RIGHT_ARMED_VAL = 0.65;

    private double LEFT_POWER = 0.65;
    private double RIGHT_POWER = 0.65;

    private volatile double LEFT_RPM = 0;
    private volatile double RIGHT_RPM = 0;

    private double MAX_RPM_DIFF = 1.5;
    private int LOW_TARGET_RPM = 67;
    private int HIGH_TARGET_RPM = 71;
    private int TARGET_RPM = LOW_TARGET_RPM;

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor sweeper;
    private Servo capArm;
    private Servo flagServo;
    private OpticalDistanceSensor opticalDistanceSensor = null;
    private ColorSensor sweeperColorSensor;

    private boolean swap = false;
    private boolean ballSensed = false;

    private double amountOfLightDetected = 0;

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

        leftMotor = this.hardwareMap.dcMotor.get("l");
        rightMotor = this.hardwareMap.dcMotor.get("r");
        sweeper = this.hardwareMap.dcMotor.get("sweeper");
        capArm = this.hardwareMap.servo.get("capArm");
        sweeperColorSensor = this.hardwareMap.colorSensor.get("colorLegacy");
        flagServo = this.hardwareMap.servo.get("flagServo");
        opticalDistanceSensor = this.hardwareMap.opticalDistanceSensor.get("ods");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        swap = true;

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        capArm.setPosition(1);

        scheduledExecutorService.schedule(new Runnable() {
            @Override
            public void run() {
                while (!scheduledExecutorService.isShutdown())
                    amountOfLightDetected = Range.clip(opticalDistanceSensor.getLightDetected(), ODS_MIN_VALUE, ODS_MAX_VALUE);
            }
        }, 0, TimeUnit.MILLISECONDS);

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
                        /*if(LEFT_RPM - TARGET_RPM < MAX_RPM_DIFF*2)
                            LEFT_POWER += MOTOR_POWER_INCREMENT*3;
                        else
                            LEFT_POWER += MOTOR_POWER_INCREMENT;*/
                        LEFT_POWER += calcDiff(LEFT_RPM, TARGET_RPM);
                    } else if (LEFT_RPM - TARGET_RPM > MAX_RPM_DIFF){
                        /*if(LEFT_RPM - TARGET_RPM > MAX_RPM_DIFF*2)
                            LEFT_POWER -= MOTOR_POWER_INCREMENT*3;
                        else
                            LEFT_POWER -= MOTOR_POWER_INCREMENT;*/
                        LEFT_POWER -= calcDiff(LEFT_RPM, TARGET_RPM);
                    }

                    if(RIGHT_RPM - TARGET_RPM < MAX_RPM_DIFF){
                        /*if(RIGHT_RPM - TARGET_RPM < MAX_RPM_DIFF*2)
                            RIGHT_POWER += MOTOR_POWER_INCREMENT*3;
                        else
                            RIGHT_POWER += MOTOR_POWER_INCREMENT;*/
                        RIGHT_POWER += calcDiff(RIGHT_RPM, TARGET_RPM);
                    } else if (RIGHT_RPM - TARGET_RPM > MAX_RPM_DIFF){
                        /*if(RIGHT_RPM - TARGET_RPM > MAX_RPM_DIFF*2)
                            RIGHT_POWER -= MOTOR_POWER_INCREMENT*3;
                        else
                            RIGHT_POWER -= MOTOR_POWER_INCREMENT;*/
                        RIGHT_POWER -= calcDiff(RIGHT_RPM, TARGET_RPM);
                    }

                    LEFT_POWER = Range.clip(LEFT_POWER, 0.38, 1);
                    RIGHT_POWER = Range.clip(RIGHT_POWER, 0.38, 1);
                }
            }

            private double calcDiff(double value, double target){
                if(Math.abs(value - target) > MAX_RPM_DIFF*2)
                    return MOTOR_POWER_INCREMENT*3;
                if(Math.abs(value - target) < MAX_RPM_DIFF*1.5)
                    return MOTOR_POWER_INCREMENT*2;
                else
                    return MOTOR_POWER_INCREMENT;
            }
        }, 0, 50, TimeUnit.MILLISECONDS);
    }

    @Override
    public void loop() {
        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;


        if (swap) {
            double temp = left;
            left = right;
            right = temp;
        }

        left = scaleInput(left);
        right = scaleInput(right);

        leftMotor.setPower(left);
        rightMotor.setPower(right);

        if (gamepad1.dpad_down) {
            leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            swap = false;
        } else if (gamepad1.dpad_up) {
            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            swap = true;
        }

        if(gamepad1.x){
            capArm.setPosition(0.70);
        }

        //ods value without ball is 0.32 for getLightDetected()
        //the max value is 0.6
        flagServo.setPosition((((ODS_MAX_VALUE-amountOfLightDetected)/(ODS_MAX_VALUE-ODS_MIN_VALUE))+1)/2);

        if (gamepad2.right_bumper) {
            sweeper.setPower(SWEEPER_OUT_POWER);
        } else if (gamepad2.right_trigger > 0.5) {
            sweeper.setPower(SWEEPER_IN_POWER);
        } else {
            sweeper.setPower(ZERO_POWER);

        }


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

        telemetry.addData("target rpm", TARGET_RPM);
        telemetry.addData("shooter power", SHOOTER_POWER);
        telemetry.addData("left power", LEFT_POWER);
        telemetry.addData("right power", RIGHT_POWER);
        telemetry.addData("left rpm", LEFT_RPM);
        telemetry.addData("right rpm", RIGHT_RPM);
        telemetry.addData("left pos", leftShooter.getCurrentPosition());
        telemetry.addData("right pos", rightShooter.getCurrentPosition());
        telemetry.addData("ods", opticalDistanceSensor.getLightDetected());
        telemetry.update();
    }

    /*
 * This method scales the joystick input so for low joystick values, the
 * scaled value is less than linear.  This is to make it easier to drive
 * the robot more precisely at slower speeds.
 */
    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}
