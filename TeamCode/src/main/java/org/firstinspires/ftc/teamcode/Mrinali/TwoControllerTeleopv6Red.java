package org.firstinspires.ftc.teamcode.Mrinali;

import android.media.MediaPlayer;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Sam.shooter.MotorFactory;
import org.firstinspires.ftc.teamcode.Sam.shooter.RPMThreadMilliseconds;
import org.firstinspires.ftc.teamcode.Sam.shooter.beans.ShooterMotor;
import org.firstinspires.ftc.teamcode.Sam.shooter.power.PowerManager;
import org.firstinspires.ftc.teamcode.Sam.shooter.util.Constants;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.AllianceColor;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;


@TeleOp(name = "Two Controller Teleopv6 Red", group = "Teleop")
//@Disabled
public class TwoControllerTeleopv6Red extends OpMode {
    private final double SWEEPER_IN_POWER = -0.7;
    private final double SWEEPER_OUT_POWER = 0.7;
    private final double MAX_POWER = 1.0;
    private final double MIN_POWER = -1.0;
    private final double ZERO_POWER = 0.0;
    private final double LEFT_IN_VAL = 0.56;
    private final double RIGHT_IN_VAL = 0.34;
    private final double LEFT_OUT_VAL = 0.12;
    private final double RIGHT_OUT_VAL = 0.76;
    private final double SERVO_ADJUSTMENT_VAL_LEFT = (Math.abs(LEFT_IN_VAL - LEFT_OUT_VAL) / 14);
    private final double SERVO_ADJUSTMENT_VAL_RIGHT = (Math.abs(RIGHT_IN_VAL - RIGHT_OUT_VAL) / 14);
    private final double SERVO_ADJUSTMENT_VAL_CAP = 0.02;
    private final double ODS_MAX_VALUE = 0.44;
    private final double ODS_MIN_VALUE = 0.32;
    double leftServoPos = 0.12;
    double rightServoPos = 0.76;
    double capServoPos = 0.38;
    private static double SHOOTER_POWER = 0.5;
    double ballRemovingTime = 0.5;


    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor scooper;
    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor sweeper;
    private Servo leftArm;
    private Servo rightArm;
    private Servo capArm;
    private Servo flagServo;
    private OpticalDistanceSensor opticalDistanceSensor = null;
    private PowerManager leftShooterPowerMgr;
    private PowerManager rightShooterPowerMgr;


    private boolean swap = false;
    private MediaPlayer wrongBallSound = null, correctBallSound = null;
    private ColorSensor sweeperColorSensor;
    private AllianceColor allianceColor = null;

    private boolean ballSensed = false;
    ElapsedTime ballTime;

    private ElapsedTime runtime = new ElapsedTime();


    private ScheduledExecutorService scheduledThreadPool = Executors.newScheduledThreadPool(200);
    private ScheduledExecutorService executorService = Executors.newScheduledThreadPool(3);
    private ScheduledFuture scheduledFuture = null;
    private ShooterMotor leftShooter, rightShooter;
    private boolean alreadyScheduled = false;
    private double amountOfLightDetected = 0;

    @Override
    public void init() {
        leftMotor = this.hardwareMap.dcMotor.get("l");
        rightMotor = this.hardwareMap.dcMotor.get("r");
        scooper = this.hardwareMap.dcMotor.get("scooper");
        shooter1 = this.hardwareMap.dcMotor.get("shooter1");
        shooter2 = this.hardwareMap.dcMotor.get("shooter2");
        sweeper = this.hardwareMap.dcMotor.get("sweeper");
        leftArm = this.hardwareMap.servo.get("leftservo");
        rightArm = this.hardwareMap.servo.get("rightservo");
        capArm = this.hardwareMap.servo.get("capArm");
        sweeperColorSensor = this.hardwareMap.colorSensor.get("colorLegacy");
        flagServo = this.hardwareMap.servo.get("flagServo");
        opticalDistanceSensor = this.hardwareMap.opticalDistanceSensor.get("ods");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        swap = true;

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wrongBallSound = MediaPlayer.create(this.hardwareMap.appContext, R.raw.police_siren);
        correctBallSound = MediaPlayer.create(this.hardwareMap.appContext, R.raw.super_mario_power_up);

        leftArm.setPosition(leftServoPos);
        rightArm.setPosition(rightServoPos);


        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorFactory motorFactory = MotorFactory.getInstance();
        leftShooter = new ShooterMotor();
        leftShooter.setName(Constants.MOTORNAME.LEFT_SHOOTER);
        motorFactory.addMotor(leftShooter);

        rightShooter = new ShooterMotor();
        rightShooter.setName(Constants.MOTORNAME.RIGHT_SHOOTER);
        motorFactory.addMotor(rightShooter);

        capArm.setPosition(1);

        leftShooterPowerMgr = new PowerManager(Constants.MOTORNAME.LEFT_SHOOTER, shooter1, this);
        rightShooterPowerMgr = new PowerManager(Constants.MOTORNAME.RIGHT_SHOOTER, shooter2, this);

        ballTime = new ElapsedTime((long)ballRemovingTime);

        for(int i = 0; i <  200; i++){
            //run a thread every fifty milliseconds, and each thread will re-run after a second
            scheduledThreadPool.scheduleAtFixedRate(new RPMThreadMilliseconds(shooter1, Constants.MOTORNAME.LEFT_SHOOTER), i * 10, Constants.ONE_SECOND, TimeUnit.MILLISECONDS);
            scheduledThreadPool.scheduleAtFixedRate(new RPMThreadMilliseconds(shooter2, Constants.MOTORNAME.RIGHT_SHOOTER), i * 10, Constants.ONE_SECOND, TimeUnit.MILLISECONDS);
        }

        executorService.schedule(new Runnable() {
            @Override
            public void run() {
                while (!executorService.isShutdown())
                    amountOfLightDetected = Range.clip(opticalDistanceSensor.getLightDetected(), ODS_MIN_VALUE, ODS_MAX_VALUE);
            }
        }, 0, TimeUnit.MILLISECONDS);

        allianceColor = AllianceColor.RED;

    }

    @Override
    public void start() {
        runtime.reset();
        super.start();
    }

    @Override
    public void loop() {
        DbgLog.msg(">LEFT RPM SHOOTER: " + leftShooter.getRpm());
        DbgLog.msg(">RIGHT RPM SHOOTER: " + rightShooter.getRpm());
        DbgLog.msg(">BATTERY LEVEL: " + hardwareMap.voltageSensor.get("sweepec").getVoltage());

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

        if(gamepad1.x){
            capArm.setPosition(0.70);
        }


        if (gamepad2.left_trigger > 0) {
            scooper.setPower(MAX_POWER);

            leftServoPos = LEFT_OUT_VAL;//if we are running the chain up, then extend the servos so they don't break
            rightServoPos = RIGHT_OUT_VAL;//if we are running the chain up, then extend the servos so they don't break
            leftArm.setPosition(leftServoPos);
            rightArm.setPosition(rightServoPos);
        } else if (gamepad2.left_bumper) {
            scooper.setPower(MIN_POWER);
        } else if (!isWrongBall()) {
            scooper.setPower(ZERO_POWER);

        }

        //ods value without ball is 0.32 for getLightDetected()
        //the max value is 0.6
        flagServo.setPosition((((ODS_MAX_VALUE-amountOfLightDetected)/(ODS_MAX_VALUE-ODS_MIN_VALUE))+1)/2);

        if (gamepad2.a) {
            if(!alreadyScheduled) {
                scheduledFuture = executorService.scheduleAtFixedRate(new Runnable() {
                    @Override
                    public void run() {
                        DbgLog.msg("REGULATING POWER!!!!!");
                        leftShooterPowerMgr.regulatePower();
                        rightShooterPowerMgr.regulatePower();
                    }
                }, 0, 100, TimeUnit.MILLISECONDS);
                alreadyScheduled = true;
            }
        } else if (gamepad2.b) {
            shooter1.setPower(SHOOTER_POWER);
            shooter2.setPower(SHOOTER_POWER);
        } else if(gamepad2.y) {
            shooter1.setPower(-SHOOTER_POWER/4);
            shooter2.setPower(-SHOOTER_POWER/4);
            //Constants.REQUESTED_ETPS = 83160;//35150;//1855;//1590;//1750 good for close shots
            //Constants.DEFAULT_POWER = 0.45;//0.51;//0.455;//0.42
            leftShooterPowerMgr.reset();
        } else if(gamepad2.x){
            shooter1.setPower(-SHOOTER_POWER);
            shooter2.setPower(-SHOOTER_POWER);
            // Constants.REQUESTED_ETPS = 77616;//1855;//1590;//1750 good for close shots
            // Constants.DEFAULT_POWER = 0.42;//0.455;//0.42
            // leftShooterPowerMgr.reset();
        }else{
                shooter1.setPower(0);
                shooter2.setPower(0);
                leftShooterPowerMgr.reset();
            rightShooterPowerMgr.reset();
            if(scheduledFuture != null)
                scheduledFuture.cancel(true);
            alreadyScheduled = false;
        }

        if(gamepad2.left_stick_y > 0.5){
            SHOOTER_POWER = SHOOTER_POWER + 0.05;
            SHOOTER_POWER = Range.clip(SHOOTER_POWER, 0, 1);
        }

        if(gamepad2.left_stick_y < -0.5){
            SHOOTER_POWER = SHOOTER_POWER - 0.05;
            SHOOTER_POWER = Range.clip(SHOOTER_POWER, 0, 1);
        }

        if(gamepad2.left_stick_y > 0.3) {
            //SHOOTER_POWER = SHOOTER_POWER + 0.03;
            SHOOTER_POWER = Range.clip(SHOOTER_POWER, 0, 1);
        } else if (gamepad2.left_stick_y < -0.3){
            //SHOOTER_POWER = SHOOTER_POWER - 0.03;
            SHOOTER_POWER = Range.clip(SHOOTER_POWER, 0, 1);
        }

        if (gamepad2.start) {
            shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        if (gamepad2.right_bumper) {
            sweeper.setPower(SWEEPER_OUT_POWER);

            setAllianceColor();
        } else if (gamepad2.right_trigger > 0.5) {
            sweeper.setPower(SWEEPER_IN_POWER);
        } else if (!isWrongBall()){
            sweeper.setPower(ZERO_POWER);
        }

        if (isWrongBall()) {
            ballTime.reset();
            telemetry.addLine("wrong ball");
            sweeper.setPower(SWEEPER_OUT_POWER);
            scooper.setPower(MAX_POWER);
        } else if (ballTime.seconds() < ballRemovingTime) {
            telemetry.addLine("sweeping ball out");
            sweeper.setPower(SWEEPER_OUT_POWER);
            scooper.setPower(MAX_POWER);
        }

        if (isWrongBall()) {
            if (!wrongBallSound.isPlaying()) {
                wrongBallSound.release();
                wrongBallSound = MediaPlayer.create(this.hardwareMap.appContext, R.raw.police_siren);
                wrongBallSound.start();
            }
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
        }


        if (gamepad2.dpad_down) {
            leftServoPos -= SERVO_ADJUSTMENT_VAL_LEFT;
            rightServoPos += SERVO_ADJUSTMENT_VAL_RIGHT;
            leftServoPos = Range.clip(leftServoPos, LEFT_OUT_VAL, LEFT_IN_VAL);//clip the range so it won't go over 1 or under 0
            rightServoPos = Range.clip(rightServoPos, RIGHT_IN_VAL, RIGHT_OUT_VAL);//clip the range so it won't go over 1 or under 0
            leftArm.setPosition(leftServoPos);
            rightArm.setPosition(rightServoPos);

        } else if (gamepad2.dpad_up) {
            leftServoPos += SERVO_ADJUSTMENT_VAL_LEFT;
            rightServoPos -= SERVO_ADJUSTMENT_VAL_RIGHT;
            leftServoPos = Range.clip(leftServoPos, LEFT_OUT_VAL, LEFT_IN_VAL);//clip the range so it won't go over 1 or under 0
            rightServoPos = Range.clip(rightServoPos, RIGHT_IN_VAL, RIGHT_OUT_VAL);//clip the range so it won't go over 1 or under 0
            leftArm.setPosition(leftServoPos);
            rightArm.setPosition(rightServoPos);
        } else if (gamepad2.dpad_left) {
            leftServoPos = LEFT_OUT_VAL;//if we are running the chain up, then extend the servos so they don't break
            rightServoPos = RIGHT_OUT_VAL;//if we are running the chain up, then extend the servos so they don't break
            leftArm.setPosition(leftServoPos);
            rightArm.setPosition(rightServoPos);
        } else if (gamepad2.dpad_right) {
            leftServoPos = LEFT_IN_VAL;
            rightServoPos = RIGHT_IN_VAL;
            leftArm.setPosition(leftServoPos);
            rightArm.setPosition(rightServoPos);
        }


        printTelemetry();
        telemetry.addData("cap", capServoPos);
        telemetry.addData("", "");//need to output something or else the telemetry won't update
        telemetry.addData("Current Power left", leftMotor.getPower());
        telemetry.addData("Current Power right", rightMotor.getPower());
        telemetry.addData("Current left RPM", leftShooter.getRpm());
        telemetry.addData("Current right RPM", rightShooter.getRpm());
        telemetry.addData("gamepad1.x", gamepad1.x);
        telemetry.addData("cap arm", capArm.getPosition());
        telemetry.addData("ods", opticalDistanceSensor.getLightDetected());
        telemetry.addData("alreadyScheduled", alreadyScheduled);
        telemetry.update();
    }

    private void printTelemetry() {
        if (Constants.USE_TELEMETRY) {
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
        try {
            scheduledThreadPool.awaitTermination(750, TimeUnit.MILLISECONDS);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        scheduledThreadPool.shutdownNow();

        executorService.shutdown();
        try {
            executorService.awaitTermination(750, TimeUnit.MILLISECONDS);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        executorService.shutdownNow();
        leftShooterPowerMgr.shutdown();
        rightShooterPowerMgr.shutdown();
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