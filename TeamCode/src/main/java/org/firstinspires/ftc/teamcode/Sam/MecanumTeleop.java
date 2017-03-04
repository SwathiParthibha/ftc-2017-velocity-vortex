package org.firstinspires.ftc.teamcode.Sam;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "Two Controller Teleop", group = "Teleop")
@Disabled
public class MecanumTeleop extends OpMode {
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightRearMotor;


    @Override
    public void init() {
    }

    @Override
    public void loop() {
        double magnitude = Math.sqrt(gamepad1.left_stick_x*gamepad1.left_stick_x+gamepad1.left_stick_y*gamepad1.left_stick_y);
        double direction = Math.toDegrees(Math.tan(Math.abs(gamepad1.left_stick_y/gamepad1.left_stick_x)));

        if(gamepad1.left_stick_y>0 && gamepad1.left_stick_x<0)//quadrant 2
        {
            direction=180-direction;
        }
        else if(gamepad1.left_stick_y<0 && gamepad1.left_stick_x<0)//quadrant 3
        {
            direction=180+direction;
        }
        else if(gamepad1.left_stick_y<0 && gamepad1.left_stick_x>0)//quadrant 4
        {
            direction=360-direction;
        }

        double rotation=gamepad1.right_stick_x;

        mecanumDrive_Polar(magnitude, direction, rotation, false);

        telemetry.addData("Magnitude", magnitude);
        telemetry.addData("Direction", direction);
        telemetry.addData("Rotation", rotation);
        telemetry.update();
    }


    @Override
    public void stop() {

        super.stop();
    }

    public void mecanumDrive_Polar(double magnitude, double direction, double rotation, boolean inverted) {

        magnitude = clipRange(magnitude) * Math.sqrt(2.0);
        if (inverted) {
            direction += 180.0;
            direction %= 360.0;
        }

        double dirInRad = Math.toRadians(direction + 45.0);
        double cosD = Math.cos(dirInRad);
        double sinD = Math.sin(dirInRad);


        double wheelSpeeds[] = new double[4];
        wheelSpeeds[0] = (sinD * magnitude + rotation);
        wheelSpeeds[1] = (cosD * magnitude - rotation);
        wheelSpeeds[2] = (cosD * magnitude + rotation);
        wheelSpeeds[3] = (sinD * magnitude - rotation);
        normalize(wheelSpeeds);

        for (int i = 0; i < wheelSpeeds.length; i++) {
            wheelSpeeds[i] = clipRange(wheelSpeeds[i], -1, 1);
        }

        if (leftFrontMotor != null) leftFrontMotor.setPower(wheelSpeeds[0]);
        if (rightFrontMotor != null) rightFrontMotor.setPower(wheelSpeeds[1]);
        if (leftRearMotor != null) leftRearMotor.setPower(wheelSpeeds[2]);
        if (rightRearMotor != null) rightRearMotor.setPower(wheelSpeeds[3]);


    }   //mecanumDrive_Polar


    public static double clipRange(double value, double lowLimit, double highLimit) {
        return (value < lowLimit) ? lowLimit : (value > highLimit) ? highLimit : value;
    }   //clipRange


    public static double clipRange(double value) {
        return clipRange(value, -1.0, 1.0);
    }   //clipRange

    private void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double magnitude = Math.abs(wheelSpeeds[i]);
            if (magnitude > maxMagnitude) {
                maxMagnitude = magnitude;
            }
        }

        if (maxMagnitude > 1.0) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }
    }   //normalize


}



