package org.firstinspires.ftc.teamcode.Shashank.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by spmeg on 10/22/2016.
 */
@TeleOp(name = "CompetitionTeleop", group = "Teleop")
@Disabled
public class CompetitionTeleop extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor scooper;
    private DcMotor shooter;
    private DcMotor sweeper;

    private boolean state;

    @Override
    public void init() {
        leftMotor = this.hardwareMap.dcMotor.get("l");
        rightMotor = this.hardwareMap.dcMotor.get("r");
        scooper = this.hardwareMap.dcMotor.get("scooper");
        shooter = this.hardwareMap.dcMotor.get("shooter");
        sweeper = this.hardwareMap.dcMotor.get("sweeper");

        state = false;

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;
        leftMotor.setPower(left);
        rightMotor.setPower(right);

        if(gamepad1.dpad_down){
            leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else if(gamepad1.dpad_up){
            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        if(gamepad1.dpad_right){
            sweeper.setPower(0.9);
            scooper.setPower(1);
        }

        if(gamepad1.left_trigger > 0){
            scooper.setPower(1);
        } else if(gamepad1.left_bumper){
            scooper.setPower(-0.7);
        } else {
            scooper.setPower(0);
        }

        if(gamepad1.a){
            shooter.setPower(0.7);
        }
        else if(gamepad1.b){
            shooter.setPower(0.6);
        } else{
            shooter.setPower(0);
        }

        if(gamepad1.right_bumper){
            sweeper.setPower(1);
        } else if(gamepad1.right_trigger > 0){
            sweeper.setPower(-1);
        } else {
            sweeper.setPower(0);
        }

        telemetry.addData("left joystick",  "%.2f", left);
        telemetry.addData("right joystick", "%.2f", right);
        telemetry.update();
    }
}
