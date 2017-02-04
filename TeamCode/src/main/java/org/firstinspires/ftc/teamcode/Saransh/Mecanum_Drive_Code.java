package org.firstinspires.ftc.teamcode.Saransh;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Saransh on 10/14/2016.
 */

@TeleOp(name="Basic Mecanum Code", group="MecanumBot")
@Disabled
public class Mecanum_Drive_Code extends OpMode {

    /* Declare OpMode members. */

    DcMotor frontRight;
    DcMotor backRight;
    DcMotor frontLeft;
    DcMotor backLeft;
    float X2 = 0;
    float Y1 = 0;
    float X1 = 0;

    //Deadzone
    double threshold = .2;

    //Change Speed
    double speedRatio = 1.0;

    public Mecanum_Drive_Code() {

    }

    @Override
    public void init() {

        frontRight = hardwareMap.dcMotor.get("motor_2");
        backRight = hardwareMap.dcMotor.get("motor_4");
        //frontRight.setDirection(DcMotor.Direction.REVERSE);
        //backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft = hardwareMap.dcMotor.get("motor_3");
        backLeft = hardwareMap.dcMotor.get("motor_1");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();
    }

    @Override
    public void loop() {

        if(gamepad1.dpad_up)
            frontRight.setPower(0.5);
        else
            frontRight.setPower(0.0);

        if(gamepad1.dpad_down)
            backRight.setPower(0.5);
        else
            backRight.setPower(0.0);

        if(gamepad1.dpad_left)
            frontLeft.setPower(0.5);
        else
            frontLeft.setPower(0.0);

        if(gamepad1.dpad_right)
            backLeft.setPower(0.5);
        else
            backLeft.setPower(0.0);

        if(Math.abs(gamepad1.left_stick_y) > threshold)
            Y1 = gamepad1.left_stick_y;
        else
            Y1 = 0;

        if(Math.abs(gamepad1.left_stick_x) > threshold)
            X1 = gamepad1.left_stick_x;
        else
            X1 = 0;

        if(Math.abs(gamepad1.right_stick_x) > threshold)
            X2 = gamepad1.right_stick_x;
        else
            X2 = 0;

        Y1 = (float) Range.clip(Y1, -1, 1);
        X1 = (float)Range.clip(X1, -1, 1);
        X2 = (float)Range.clip(X2, -1, 1);

        //Move the wheel Commands
        frontRight.setPower((Y1 - X2 - X1)*speedRatio);
        backRight.setPower((Y1 - X2 + X1)*speedRatio);
        frontLeft.setPower((Y1 + X2 + X1)*speedRatio);
        backLeft.setPower((Y1 + X2 - X1)*speedRatio);

        telemetry.addData("Left_Stick_Y: ", Math.abs(gamepad1.left_stick_y));
        telemetry.addData("Left_Stick_X: ", Math.abs(gamepad1.left_stick_x));
        telemetry.addData("Right_Stick_X: ", Math.abs(gamepad1.right_stick_x));
        telemetry.addData("NEW", "LINE");
        telemetry.addData("Y1: ", Y1);
        telemetry.addData("X1: ", X1);
        telemetry.addData("X2: ", X2);
        telemetry.addData("NEW", "LINE");
        telemetry.addData("frontRight: ", frontRight.getPower());
        telemetry.addData("backRight: ", backRight.getPower());
        telemetry.addData("frontLeft: ", frontLeft.getPower());
        telemetry.addData("backLeft: ", backLeft.getPower());

        telemetry.update();
    }
    @Override
    public void stop() {

    }
}
