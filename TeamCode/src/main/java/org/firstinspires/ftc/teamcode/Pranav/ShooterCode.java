package org.firstinspires.ftc.teamcode.Pranav;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

/**
 * Created by spmeg on 10/22/2016.
 */
@TeleOp(name = "DrRyanMecanumTestShooterTeleOp", group = "Teleop")
//@Disabled
public class ShooterCode extends OpMode
{
    DcMotor shooterMotorLeft;
    DcMotor shooterMotorRight;

    Servo shooterServo;
    Servo angularServo;
    Servo turretServo;

    static final double PARTICLE_HOLDER_RESET_POSITION = 0.614;
    static final double PARTICLE_HOLDER_ARMED_POSITION = 0.577;
    static final double PARTICLE_HOLDER_SHOOT_POSITION = 0.398;

    final double AZIMUTH_STOP = 0.455;

    @Override
    public void init()
    {
        shooterMotorRight = hardwareMap.dcMotor.get("shooterMotorRight");
        shooterMotorLeft = hardwareMap.dcMotor.get("shooterMotorLeft");

        shooterServo = hardwareMap.servo.get("shooterServo");
        angularServo = hardwareMap.servo.get("angularServo");
        turretServo = hardwareMap.servo.get("continue");

        shooterMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        shooterServo.setPosition(PARTICLE_HOLDER_ARMED_POSITION);
        angularServo.setPosition(0.2);
        turretServo.setPosition(AZIMUTH_STOP);

    }

    @Override
    public void loop()
    {
        if (gamepad1.b)
        {
            shooterServo.setPosition(PARTICLE_HOLDER_SHOOT_POSITION);

            try {
                sleep(250);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            shooterServo.setPosition(PARTICLE_HOLDER_ARMED_POSITION);

            try
            {
                sleep(250);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        shooterMotorRight.setPower(1);////;
        shooterMotorLeft.setPower(1);

    }


}
