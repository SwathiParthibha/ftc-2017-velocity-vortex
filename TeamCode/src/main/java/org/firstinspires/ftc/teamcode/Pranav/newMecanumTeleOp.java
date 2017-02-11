package org.firstinspires.ftc.teamcode.Pranav;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static android.os.Build.VERSION_CODES.M;

/**
 * Created by spmeg on 10/22/2016.
 */
@TeleOp(name = "New Mecanum TeleOp", group = "Teleop")
public class newMecanumTeleOp extends OpMode
{
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

    GyroSensor sensorGyro;

    double K;
    double forward;
    double right;
    double clockwise;
    double temp;
    double theta = sensorGyro.getHeading();

    double speedFrontLeftMotor;
    double speedFrontRightMotor;
    double speedBackLeftMotor;
    double speedBackRightMotor;

    @Override
    public void init()
    {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        sensorGyro = hardwareMap.gyroSensor.get("sensorGyro");
    }

    @Override
    public void loop()
    {
        K = 0;
        forward = gamepad1.left_stick_y;
        right = gamepad1.left_stick_x;
        clockwise = gamepad1.right_stick_x;

        clockwise = K*clockwise;

        //Clockwise from Zero Reference
        temp = forward * Math.cos(theta) + right * Math.sin(theta);
        right = -forward * Math.sin(theta) + right * Math.cos(theta);
        forward = temp;

        /*
        //Counter Clockwise from Zero Reference
        temp = forward * Math.cos(theta) - right * Math.sin(theta);
        right = forward * Math.sin(theta) + right * Math.cos(theta);
        forward = temp;
        */

        speedFrontLeftMotor = forward + clockwise + right;
        speedFrontRightMotor = forward - clockwise - right;
        speedBackLeftMotor = forward + clockwise - right;
        speedBackRightMotor = forward - clockwise + right;

        speedFrontLeftMotor = Range.clip(speedFrontLeftMotor, -1, 1);
        speedFrontRightMotor = Range.clip(speedFrontRightMotor, -1, 1);
        speedBackLeftMotor = Range.clip(speedBackLeftMotor, -1, 1);
        speedBackRightMotor = Range.clip(speedBackRightMotor, -1, 1);

        frontLeftMotor.setPower(speedFrontLeftMotor);
        frontRightMotor.setPower(speedFrontRightMotor);
        backLeftMotor.setPower(speedBackLeftMotor);
        backRightMotor.setPower(speedBackRightMotor);

        telemetry.addData("Left Joystick Y Direction:", gamepad1.left_stick_y);
        telemetry.addData("Left Joystick X Direction:", gamepad1.left_stick_x);
        telemetry.addData("Right Joystick X Direction:", gamepad1.right_stick_x);
        telemetry.addData("Theta:", theta);

        telemetry.addData("Front Left ShooterMotor Speed:", frontLeftMotor.getPower());
        telemetry.addData("Front Right ShooterMotor Speed:", frontRightMotor.getPower());
        telemetry.addData("Back Left ShooterMotor Speed:", backLeftMotor.getPower());
        telemetry.addData("Back Right ShooterMotor Speed:", backRightMotor.getPower());

    }




    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

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



