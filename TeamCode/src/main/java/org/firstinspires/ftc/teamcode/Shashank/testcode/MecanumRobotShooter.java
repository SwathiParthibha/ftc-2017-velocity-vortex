package org.firstinspires.ftc.teamcode.Shashank.testcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;

import hallib.HalDashboard;

/**
 * Created by spmeg on 4/13/2017.
 */
@TeleOp(name = "MecanumRobotShooter", group = "TESTCODE")
public class MecanumRobotShooter extends OpMode implements Runnable{
    private final double LOADED_POSITION_TRIGGER = 0.557;
    private final double SHOOT_POSITION_TRIGGER = 0.428;
    private final double START_POSITION_ANGULAR = 0.3;
    private final double START_POSITION_TURRET = 0.457;
    private final double SHOOTER_POWER = 1.0;
    private final double REVERSE_SHOOTER_POWER = -0.5;

    private Servo triggerServo, angularServo, turretServo;

    private boolean OP_MODE_IS_ACTIVE = true;
    private boolean SHOOTER_MOTORS_ACTIVE = false;
    private boolean SHOOTER_MOTORS_REVERSE = false;
    private boolean SHOOTER_MOTORS_IS_READY = false;

    private HalDashboard halDashboard = null;
    private final int LABEL_WIDTH = 200;
    private DcMotor leftShooter, rightShooter;

    boolean               inputPin;             // Input State
    DigitalChannel        digIn;                // Device Object

    @Override
    public void init() {
        halDashboard = HalDashboard.createInstance(telemetry);

        triggerServo = hardwareMap.servo.get("triggerServo");
        angularServo = hardwareMap.servo.get("pixyyaxis");
        turretServo = hardwareMap.servo.get("pixyyrotation");

        leftShooter = hardwareMap.dcMotor.get("shooterLeft");
        rightShooter = hardwareMap.dcMotor.get("shooterRight");

        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        digIn  = hardwareMap.get(DigitalChannel.class, "digin");     //  Use generic form of device mapping
        digIn.setMode(DigitalChannelController.Mode.INPUT);          // Set the direction of each channel

        triggerServo.setPosition(LOADED_POSITION_TRIGGER);
        angularServo.setPosition(START_POSITION_ANGULAR);
        turretServo.setPosition(START_POSITION_TURRET);


        new Thread(this).start();
    }

    @Override
    public void loop() {

        if(SHOOTER_MOTORS_REVERSE){
            //leftShooter.setPower(REVERSE_SHOOTER_POWER);
            //rightShooter.setPower(REVERSE_SHOOTER_POWER);
        } else if (SHOOTER_MOTORS_ACTIVE){
            //leftShooter.setPower(SHOOTER_POWER);
            //rightShooter.setPower(SHOOTER_POWER);
        } else {
            leftShooter.setPower(0);
            rightShooter.setPower(0);
        }

        halDashboard.displayPrintf(1, LABEL_WIDTH, "digital input: ", "%b", inputPin);
        halDashboard.displayPrintf(2, LABEL_WIDTH, "servo position: ", "%1.3f", triggerServo.getPosition());
    }

    @Override
    public void stop() {
        super.stop();
        OP_MODE_IS_ACTIVE = false;
    }

    //THE THREAD THAT WILL RUN THE TRIGGER
    @Override
    public void run() {
        while (OP_MODE_IS_ACTIVE){
            inputPin = digIn.getState();
            if(!gamepad1.b) {
                SHOOTER_MOTORS_REVERSE = false;
                if (inputPin && gamepad1.a) {

                    SHOOTER_MOTORS_ACTIVE = true;

                    //if the shooter motors are not ready, SPEED THEM UP
                    if (!SHOOTER_MOTORS_IS_READY) {
                        for (long i = System.currentTimeMillis(); System.currentTimeMillis() - i < 250; ) {
                            SHOOTER_MOTORS_ACTIVE = true;
                        }

                        //notify opmode that shooter motors are ready
                        SHOOTER_MOTORS_IS_READY = true;
                    }

                    for (long i = System.currentTimeMillis(); System.currentTimeMillis() - i < 450; ) {
                        triggerServo.setPosition(SHOOT_POSITION_TRIGGER);
                    }
                } else if (inputPin) {

                    SHOOTER_MOTORS_ACTIVE = true;

                    //if the shooter motors are not ready, SPEED THEM UP
                    if (!SHOOTER_MOTORS_IS_READY) {
                        for (long i = System.currentTimeMillis(); System.currentTimeMillis() - i < 250; ) {
                            SHOOTER_MOTORS_ACTIVE = true;
                        }
                        //notify opmode that shooter motors are ready
                        SHOOTER_MOTORS_IS_READY = true;
                    }

                    for (long i = System.currentTimeMillis(); System.currentTimeMillis() - i < 250; ) {
                        triggerServo.setPosition(LOADED_POSITION_TRIGGER);
                    }
                } else {
                    //SHOOTER_MOTORS_ACTIVE = false;
                    //SHOOTER_MOTORS_IS_READY = false;

                    SHOOTER_MOTORS_ACTIVE = true;
                    SHOOTER_MOTORS_IS_READY = true;

                    for (long i = System.currentTimeMillis(); System.currentTimeMillis() - i < 250; ) {
                        triggerServo.setPosition(LOADED_POSITION_TRIGGER);
                    }
                }
            } else if (gamepad1.b) {
                SHOOTER_MOTORS_ACTIVE = false;
                SHOOTER_MOTORS_IS_READY = false;
                SHOOTER_MOTORS_REVERSE = true;
            }
        }
    }
}
