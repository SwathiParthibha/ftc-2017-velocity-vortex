package org.firstinspires.ftc.teamcode.Sam;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Sam.shooter.MotorFactory;
import org.firstinspires.ftc.teamcode.Sam.shooter.RPMThreadMilliseconds;
import org.firstinspires.ftc.teamcode.Sam.shooter.beans.ShooterMotor;
import org.firstinspires.ftc.teamcode.Sam.shooter.power.PowerManager;
import org.firstinspires.ftc.teamcode.Sam.shooter.util.Constants;
import org.firstinspires.ftc.teamcode.Sam.util.Util;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@Disabled
@TeleOp(name = "twoControllerTeleop5", group = "Teleop")
public class twoControllerTeleopv5 extends OpMode {
    private final double SWEEPER_IN_POWER = -0.7;
    private final double SWEEPER_OUT_POWER = 0.7;
    private final double MAX_POWER = 1.0;
    private final double MIN_POWER = -1.0;
    private final double ZERO_POWER = 0.0;


    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor scooper;
    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor sweeper;
    private PowerManager leftShooterPowerMgr;
    private PowerManager rightShooterPowerMgr;

    private boolean swap = false;
    private MediaPlayer wrongBallSound = null, correctBallSound = null;
    private ColorSensor sweeperColorSensor;
    private org.firstinspires.ftc.teamcode.Shashank.statemachine.AllianceColor allianceColor = null;

    private boolean ballSensed = false;

    private ElapsedTime runtime = new ElapsedTime();


    private ScheduledExecutorService scheduledThreadPool = Executors.newScheduledThreadPool(2);

    @Override
    public void init() {
        leftMotor = this.hardwareMap.dcMotor.get("l");
        rightMotor = this.hardwareMap.dcMotor.get("r");
        scooper = this.hardwareMap.dcMotor.get("scooper");
        shooter1 = this.hardwareMap.dcMotor.get("shooter1");
        shooter2 = this.hardwareMap.dcMotor.get("shooter2");
        sweeper = this.hardwareMap.dcMotor.get("sweeper");
        sweeperColorSensor = this.hardwareMap.colorSensor.get("colorLegacy");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        swap = true;

        wrongBallSound = MediaPlayer.create(this.hardwareMap.appContext, R.raw.police_siren);
        correctBallSound = MediaPlayer.create(this.hardwareMap.appContext, R.raw.super_mario_power_up);


        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorFactory motorFactory = MotorFactory.getInstance();
        ShooterMotor leftShooter = new ShooterMotor();
        leftShooter.setName(Constants.MOTORNAME.LEFT_SHOOTER);
        motorFactory.addMotor(leftShooter);

        ShooterMotor rightShooter = new ShooterMotor();
        rightShooter.setName(Constants.MOTORNAME.RIGHT_SHOOTER);
        motorFactory.addMotor(rightShooter);

        leftShooterPowerMgr = new PowerManager(Constants.MOTORNAME.LEFT_SHOOTER, shooter1, this);
        rightShooterPowerMgr = new PowerManager(Constants.MOTORNAME.RIGHT_SHOOTER, shooter2, this);

        scheduledThreadPool.scheduleAtFixedRate(new RPMThreadMilliseconds(shooter1, Constants.MOTORNAME.LEFT_SHOOTER), 0L, Constants.DELTA_TIME, TimeUnit.MILLISECONDS);
        scheduledThreadPool.scheduleAtFixedRate(new RPMThreadMilliseconds(shooter2, Constants.MOTORNAME.RIGHT_SHOOTER), 0L, Constants.DELTA_TIME, TimeUnit.MILLISECONDS);
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


        if (swap) {
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
            scooper.setPower(SWEEPER_IN_POWER);
        } else if (gamepad2.left_bumper) {
            scooper.setPower(MAX_POWER);
        } else {
            scooper.setPower(ZERO_POWER);

        }

        if (gamepad2.a) {
            leftShooterPowerMgr.regulatePower();
            rightShooterPowerMgr.regulatePower();
        } else {
            shooter1.setPower(0);
            shooter2.setPower(0);
        }


        if (gamepad2.right_bumper) {
            sweeper.setPower(SWEEPER_OUT_POWER);

            setAllianceColor();
        } else if (gamepad2.right_trigger > 0) {
            sweeper.setPower(SWEEPER_OUT_POWER);
        } else {
            sweeper.setPower(ZERO_POWER);

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

        printTelemetry();
        telemetry.addData("", "");//need to output something or else the telemetry won't update
        telemetry.update();
    }

    private void printTelemetry() {
        if (Constants.USE_TELEMETRY) {
            telemetry.addData("Hello", "Here");
            telemetry.addData("", leftShooterPowerMgr.getMotorTelemetry().toString());
            telemetry.addData("", rightShooterPowerMgr.getMotorTelemetry().toString());
        }
    }

    private void setAllianceColor() {
        if (allianceColor != null) {
            return;
        }
        if (sweeperColorSensor.red() > 15) {
            if (sweeperColorSensor.red() > sweeperColorSensor.blue())
                allianceColor = allianceColor.RED;
        } else if (sweeperColorSensor.blue() > 15) {
            if (sweeperColorSensor.red() < sweeperColorSensor.blue())
                allianceColor = allianceColor.BLUE;
        } else {
            allianceColor = allianceColor.BLUE;
        }
        telemetry.log().add("Beacon Color Set");
    }

    private boolean isWrongBall() {
        if (sweeperColorSensor.red() > 11 || sweeperColorSensor.blue() > 11) {
            ballSensed = true;
            if (sweeperColorSensor.red() > sweeperColorSensor.blue() && allianceColor == allianceColor.BLUE) {
                return true;
            } else if (sweeperColorSensor.blue() > sweeperColorSensor.red() && allianceColor == allianceColor.RED) {
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