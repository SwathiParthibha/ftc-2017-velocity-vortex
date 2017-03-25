package org.firstinspires.ftc.teamcode.Pranav;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp(name = "Mecanum TeleOp", group = "TeleOp")
public class newMecanumTeleOp extends OpMode
{
    DcMotor shooterMotorRight;
    DcMotor shooterMotorLeft;

    Servo shooterServo;


    double RESET_POSITION = 0.614;
    double ARMED_POSITION = 0.577;
    double SHOOT_POSITION = 0.398;

    double shooterServoPosition = 0;


    @Override
    public void init()
    {

        shooterServo = hardwareMap.servo.get("shooterServo");
        shooterServo.setPosition(RESET_POSITION);
    }

    @Override
    public void loop()
    {
        shooterServo.setPosition(shooterServoPosition);


        if(gamepad1.a)
        {
            shooterServoPosition = RESET_POSITION;
        }

        if(gamepad1.b)
        {
            shooterServoPosition = SHOOT_POSITION;
        }

        telemetry.addData("Shooter Servo", shooterServo.getPosition());
    }


    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)
    {
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



