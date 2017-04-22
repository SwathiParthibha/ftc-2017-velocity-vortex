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

import hallib.HalDashboard;
import trclib.TrcKalmanFilter;

/**
 * Created by spmeg on 4/20/2017.
 */
@TeleOp(name = "TestCodeShooter", group = "Test")
public class TestCodeShooter extends OpMode {
    private DcMotor leftShooter, rightShooter;
    private double MOTOR_POWER_INCREMENT = 0.0005;
    private double SHOOTER_POWER = 0.5;
    private VoltageSensor voltageSensor = null;
    private DcMotor scooper;
    private Servo leftArm;
    private Servo rightArm;
    private double k = 9.5423562F;

    private final double MAX_POWER = 1.0;
    private final double MIN_POWER = -1.0;
    private final double ZERO_POWER = 0.0;

    private final double LEFT_IN_VAL = 0.56;
    private final double RIGHT_IN_VAL = 0.34;
    private final double LEFT_OUT_VAL = 0.12;
    private final double RIGHT_OUT_VAL = 0.76;
    private double calculatedValue = 0.0;

    private ScheduledExecutorService scheduledExecutorService = Executors.newSingleThreadScheduledExecutor();

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
    }

    @Override
    public void loop() {
        SHOOTER_POWER = k/voltageSensor.getVoltage();
        SHOOTER_POWER = Range.clip(SHOOTER_POWER, 0.4, 1);

        if(gamepad2.x){
            calculatedValue = k/voltageSensor.getVoltage();
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

        if (gamepad2.dpad_left) {
            leftArm.setPosition(LEFT_OUT_VAL);
            rightArm.setPosition(RIGHT_OUT_VAL);
        } else if (gamepad2.dpad_right) {
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
            leftShooter.setPower(calculatedValue);
            rightShooter.setPower(calculatedValue);
        } else {
            leftShooter.setPower(0);
            rightShooter.setPower(0);
        }

        if(gamepad2.y){
            telemetry.addData("K VALUE", SHOOTER_POWER*voltageSensor.getVoltage());
        }

        if(k/voltageSensor.getVoltage() > 1.0){
            telemetry.addData("WARNING", "voltage too low");
        }
        telemetry.addData("calculated power", calculatedValue);
        telemetry.addData("voltage", voltageSensor.getVoltage());
        telemetry.addData("shooter power", SHOOTER_POWER);
        telemetry.update();
    }
}
