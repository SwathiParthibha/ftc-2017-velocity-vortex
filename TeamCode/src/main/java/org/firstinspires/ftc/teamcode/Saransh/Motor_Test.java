package org.firstinspires.ftc.teamcode.Saransh;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Saransh on 10/22/2016.
 */
@TeleOp(name="Motor Test", group="Tests")
@Disabled
public class Motor_Test extends OpMode {

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

    public Motor_Test() {

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


        if (gamepad1.b)
            frontRight.setPower(0.7);
        else
            frontRight.setPower(0.0);
        if (gamepad1.x)
            backRight.setPower(0.7);
        else
            backRight.setPower(0.0);

        if (gamepad1.dpad_up)
            frontLeft.setPower(0.7);
        else
            frontLeft.setPower(0.0);
        if (gamepad1.dpad_down)
            backLeft.setPower(0.7);
        else
            backLeft.setPower(0.0);

        telemetry.addData("gamepad1.x", gamepad1.x);
        telemetry.addData("gamepad1.b", gamepad1.b);
        telemetry.addData("gamepad1.dpad_up", gamepad1.dpad_up);
        telemetry.addData("gamepad1.dpad_down", gamepad1.dpad_down);

        telemetry.addData("frontRight: ", frontRight.getPower());
        telemetry.addData("backRight: ", backRight.getPower());
        telemetry.addData("frontLeft: ", frontLeft.getPower());
        telemetry.addData("backLeft: ", backLeft.getPower());
    }
}
