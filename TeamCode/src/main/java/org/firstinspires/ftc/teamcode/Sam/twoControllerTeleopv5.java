package org.firstinspires.ftc.teamcode.Sam;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Sam.shooter.MotorFactory;
import org.firstinspires.ftc.teamcode.Sam.shooter.RPMThread;
import org.firstinspires.ftc.teamcode.Sam.shooter.beans.ShooterMotor;
import org.firstinspires.ftc.teamcode.Sam.shooter.power.PowerManager;
import org.firstinspires.ftc.teamcode.Sam.shooter.util.Constants;
import org.firstinspires.ftc.teamcode.Sam.util.Util;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;


@TeleOp(name = "twoControllerTeleop5", group = "Teleop")
public class twoControllerTeleopv5 extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor scooper;
    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor sweeper;
    private PowerManager leftShooterPowerMgr;
    private PowerManager rightShooterPowerMgr;

    boolean swap = false;
    private MediaPlayer wrongBallSound = null, correctBallSound = null;
    private ColorSensor sweeperColorSensor;
    private org.firstinspires.ftc.teamcode.Shashank.statemachine.AllianceColor AllianceColor = null;

    private boolean ballSensed = false;

    private ElapsedTime runtime = new ElapsedTime();


    private ScheduledExecutorService scheduledThreadPool = Executors.newScheduledThreadPool(2);

    @Override
    public void init() {

        shooter1 = this.hardwareMap.dcMotor.get("shooter1");
        shooter2 = this.hardwareMap.dcMotor.get("shooter2");

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorFactory motorFactory = MotorFactory.getInstance();
        ShooterMotor leftShooter = new ShooterMotor();
        leftShooter.setName(Constants.MOTORNAME.LEFT_SHOOTER);
        motorFactory.addMotor(leftShooter);

        ShooterMotor rightShooter = new ShooterMotor();
        rightShooter.setName(Constants.MOTORNAME.RIGHT_SHOOTER);
        motorFactory.addMotor(rightShooter);

        leftShooterPowerMgr = new PowerManager(Constants.MOTORNAME.LEFT_SHOOTER, shooter1);
        rightShooterPowerMgr = new PowerManager(Constants.MOTORNAME.RIGHT_SHOOTER, shooter2);

        scheduledThreadPool.scheduleAtFixedRate(new RPMThread(shooter1, Constants.MOTORNAME.LEFT_SHOOTER), 0L, Constants.DELTA_TIME, TimeUnit.MILLISECONDS);
        scheduledThreadPool.scheduleAtFixedRate(new RPMThread(shooter2, Constants.MOTORNAME.RIGHT_SHOOTER), 0L, Constants.DELTA_TIME, TimeUnit.MILLISECONDS);
    }


    @Override
    public void start() {
        runtime.reset();
        super.start();
    }

    @Override
    public void loop() {



        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;


        if (swap == true) {
            double temp = left;
            left = right;
            right = temp;
        }

        left = scaleInput(left);
        right = scaleInput(right);

        leftMotor.setPower(left);
        rightMotor.setPower(right);

        if (gamepad1.dpad_down) {
            leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            swap = false;
        } else if (gamepad1.dpad_up) {
            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            swap = true;
        }


        if (gamepad2.left_trigger > 0) {
            scooper.setPower(-0.7);
        } else if (gamepad2.left_bumper) {
            scooper.setPower(1);
        } else {
            scooper.setPower(0);
        }

        if (gamepad2.a) {
            leftShooterPowerMgr.regulatePower();
            rightShooterPowerMgr.regulatePower();
        } else {
            shooter1.setPower(0);
            shooter2.setPower(0);
        }


        if (gamepad2.right_bumper) {
            sweeper.setPower(0.7);

            if (AllianceColor == null) {
                if (sweeperColorSensor.red() > 15) {
                    if (sweeperColorSensor.red() > sweeperColorSensor.blue())
                        AllianceColor = AllianceColor.RED;
                } else if (sweeperColorSensor.blue() > 15) {
                    if (sweeperColorSensor.red() < sweeperColorSensor.blue())
                        AllianceColor = AllianceColor.BLUE;
                } else
                    AllianceColor = AllianceColor.BLUE;
                telemetry.log().add("Beacon Color Set");
            }
        } else if (gamepad2.right_trigger > 0) {
            sweeper.setPower(-0.7);
        } else {
            sweeper.setPower(0);

        }

        if (isWrongBall()) {
            if (!wrongBallSound.isPlaying()) {
                wrongBallSound.release();
                wrongBallSound = MediaPlayer.create(this.hardwareMap.appContext, R.raw.police_siren);
                wrongBallSound.start();
            }
            //sweeper.setPower(0.5);
        } else {
            isWrongBall();
            if (ballSensed) {
                correctBallSound.release();
                correctBallSound = MediaPlayer.create(this.hardwareMap.appContext, R.raw.super_mario_power_up);
                correctBallSound.start();
            } else {
                correctBallSound.stop();
            }
            wrongBallSound.stop();
            //sweeper.setPower(0);
        }

        telemetry.addData("left joystick", "%.2f", left);
        telemetry.addData("right joystick", "%.2f", right);
        telemetry.update();
    }

    private boolean isWrongBall() {
        if (sweeperColorSensor.red() > 11 || sweeperColorSensor.blue() > 11) {
            ballSensed = true;
            if (sweeperColorSensor.red() > sweeperColorSensor.blue() && AllianceColor == AllianceColor.BLUE) {
                return true;
            } else if (sweeperColorSensor.blue() > sweeperColorSensor.red() && AllianceColor == AllianceColor.RED) {
                return true;
            } else {
                return false;
            }
        } else {
            ballSensed = false;
            return false;
        }
    }

    @Override
    public void stop() {
        super.stop();

        scheduledThreadPool.shutdown();
        Util.waitUntil(5);
        scheduledThreadPool.shutdownNow();
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

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