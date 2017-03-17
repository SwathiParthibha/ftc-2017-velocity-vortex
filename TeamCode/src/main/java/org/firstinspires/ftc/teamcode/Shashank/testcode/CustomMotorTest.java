package org.firstinspires.ftc.teamcode.Shashank.testcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Sam.util.Util;

/**
 * Created by spmeg on 10/28/2016.
 */
@Autonomous(name = "CustomMotorTest", group = "Tests")
@Disabled
public class CustomMotorTest extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private ElapsedTime time = new ElapsedTime();

    @Override
    public void init() {
        leftMotor = this.hardwareMap.dcMotor.get("l");
        rightMotor = this.hardwareMap.dcMotor.get("r");

        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {

        if(time.seconds() < 4){
            leftMotor.setPower(1);
            rightMotor.setPower(1);
        } else if( time.seconds() < 8){
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        } else {
            time.reset();
        }

        if(leftMotor.getCurrentPosition() > 2000 || rightMotor.getCurrentPosition() > 2000){
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addData("leftMotor Pos", leftMotor.getCurrentPosition());
        telemetry.addData("rightMotor Pos", rightMotor.getCurrentPosition());
        telemetry.addData("leftMotor mode", leftMotor.getMode());
        telemetry.addData("rightMotor Pos", rightMotor.getMode());
        telemetry.addData("leftMotor brake type", leftMotor.getZeroPowerBehavior());
        telemetry.addData("rightMotor brake type", rightMotor.getZeroPowerBehavior());
        telemetry.update();
    }
}
