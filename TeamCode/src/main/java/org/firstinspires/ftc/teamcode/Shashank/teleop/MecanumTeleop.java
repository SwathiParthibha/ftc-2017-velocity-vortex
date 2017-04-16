package org.firstinspires.ftc.teamcode.Shashank.teleop;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.HashMap;

import ftclib.FtcDcMotor;
import hallib.HalDashboard;
import swlib.SWGamePad;
import swlib.SWIMUGyro;
import swlib.SwDriveBase;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;

/**
 * Created by spmeg on 4/15/2017.
 */
@TeleOp(name = "MecanumTeleop", group = "teleop")
public class MecanumTeleop extends OpMode  implements SWGamePad.ButtonHandler, Runnable{
    private final double LOADED_POSITION_TRIGGER = 0.557;
    private final double SHOOT_POSITION_TRIGGER = 0.428;
    private final double START_POSITION_ANGULAR = 0.3;
    private final double START_POSITION_TURRET = 0.50;
    private final double CLOCKWISE_POSITION_TURRET = 0.40;
    private final double COUNTERCLOCKWISE_POSITION_TURRET = 0.60;
    private final double SHOOTER_POWER = 1.0;
    private final double REVERSE_SHOOTER_POWER = -0.5;
    private final double SERVO_INCREMENT = 0.0025;

    private Servo triggerServo, angularServo, turretServo;

    private boolean OP_MODE_IS_ACTIVE = true;
    private boolean SHOOTER_MOTORS_ACTIVE = false;
    private boolean SHOOTER_MOTORS_REVERSE = false;
    private boolean SHOOTER_MOTORS_IS_READY = false;

    private DcMotor leftShooter, rightShooter;

    boolean               inputPin;             // Input State
    DigitalChannel digIn;                // Device Object

    private FtcDcMotor leftFrontMotor;
    private FtcDcMotor leftRearMotor;
    private FtcDcMotor rightFrontMotor;
    private FtcDcMotor rightRearMotor;
    private SwDriveBase driveBase = null;
    private SWIMUGyro gyro = null;
    private SWGamePad gamepad;
    private boolean fixedOnTarget = false;
    private HalDashboard dashboard = null;
    private final int LABEL_WIDTH = 200;
    private boolean setYInverted = true;
    private boolean setXInverted = false;
    private TrcTaskMgr taskMgr = new TrcTaskMgr();
    private HashMap<Integer, ElapsedTime> toggleTimeTracker = new HashMap<>();

    @Override
    public void init_loop() {
        super.init_loop();
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.PREPERIODIC_TASK, TrcRobot.RunMode.TELEOP_MODE);
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK, TrcRobot.RunMode.TELEOP_MODE);
    }

    @Override
    public void start() {
        super.start();
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.START_TASK, TrcRobot.RunMode.TELEOP_MODE);
    }

    @Override
    public void stop() {
        super.stop();
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.STOP_TASK, TrcRobot.RunMode.TELEOP_MODE);
        OP_MODE_IS_ACTIVE = false;
    }

    @Override
    protected void preInit() {
        super.preInit();
    }

    @Override
    protected void postInitLoop() {
        super.postInitLoop();
    }

    @Override
    protected void postLoop() {
        super.postLoop();
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.POSTPERIODIC_TASK, TrcRobot.RunMode.TELEOP_MODE);
    }

    @Override
    public void init() {
        dashboard = HalDashboard.createInstance(this.telemetry);
        DbgLog.msg("> INIT" + "STARTED INIT");

        DbgLog.msg("> INIT" + "STARTED OBJECT CREATION OF GYRO");
        gyro = new SWIMUGyro(hardwareMap, "gyro", null);
        gyro.calibrate();
        DbgLog.msg("> INIT" + "FINISHED OBJECT CREATION OF GYRO");

        DbgLog.msg("> INIT" + "GENERATING MOTORS");
        leftFrontMotor = new FtcDcMotor(this.hardwareMap, "leftFront", null, null);
        leftRearMotor = new FtcDcMotor(this.hardwareMap, "leftRear", null, null);
        rightFrontMotor = new FtcDcMotor(this.hardwareMap, "rightFront", null, null);
        rightRearMotor = new FtcDcMotor(this.hardwareMap, "rightRear", null, null);
        DbgLog.msg("> INIT" + "FINISHED GENERATING MOTORS");

        DbgLog.msg("> INIT" + "INVERTING MOTORS");
        leftFrontMotor.setInverted(true);
        leftRearMotor.setInverted(true);
        DbgLog.msg("> INIT" + "FINISHED INVERTING MOTORS");

        DbgLog.msg("> INIT" + "CHANGING MODE OF MOTORS");
        leftFrontMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DbgLog.msg("> INIT" + "FINISHED CHANGING MODE OF MOTORS");

        DbgLog.msg("> INIT" + "INITING TRCDRIVEBASE");
        driveBase = new SwDriveBase(
                leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, gyro);
        /*driveBase = new TrcDriveBase(
                leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);*/
        DbgLog.msg("> INIT" + "FINISHED INITING TRCDRIVEBASE");

        DbgLog.msg("> INIT" + "ENABLING GYRO");
        gyro.setEnabled(true);
        DbgLog.msg("> INIT" + "FINISHED ENABLING GYRO");

        DbgLog.msg("> INIT" + "CREATING GAMEPAD");
        gamepad = new SWGamePad("driver gamepad", gamepad1, 0.05F);
        gamepad.enableDebug(true);
        DbgLog.msg("> INIT" + "FINISHED CREATING GAMEPAD");

        driveBase.enableGyroAssist(0.0001, 0.05);

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

        DbgLog.msg("> INIT" + "FINISHED INIT");
    }

    @Override
    public void loop() {
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.PREPERIODIC_TASK, TrcRobot.RunMode.TELEOP_MODE);
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK, TrcRobot.RunMode.TELEOP_MODE);
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.PREPERIODIC_TASK, TrcRobot.RunMode.AUTO_MODE);
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK, TrcRobot.RunMode.AUTO_MODE);

        gamepad.setYInverted(setYInverted);
        double rotation = gamepad.getRightStickX()*-1;
        double magnitude = Range.clip(gamepad.getLeftStickMagnitude(), 0, 1);
        double direction = gamepad.getLeftStickDirectionDegrees(true);

        if(gamepad.getLeftStickX() == 0 && gamepad.getLeftStickY() == 0)
            magnitude = 0;

        driveBase.mecanumDrive_PolarFieldCentric(magnitude, direction, rotation);


        if(gamepad1.x){
            turretServo.setPosition(CLOCKWISE_POSITION_TURRET);
        } else if (gamepad1.b){
            turretServo.setPosition(COUNTERCLOCKWISE_POSITION_TURRET);
        } else {
            turretServo.setPosition(START_POSITION_TURRET);
        }

        if(gamepad1.a){
            angularServo.setPosition(Range.clip(angularServo.getPosition()-SERVO_INCREMENT, 0, 1));
        } else if (gamepad1.y){
            angularServo.setPosition(Range.clip(angularServo.getPosition()+SERVO_INCREMENT, 0, 1));
        }

        if(SHOOTER_MOTORS_REVERSE){
            leftShooter.setPower(REVERSE_SHOOTER_POWER);
            rightShooter.setPower(REVERSE_SHOOTER_POWER);
        } else if (SHOOTER_MOTORS_ACTIVE){
            leftShooter.setPower(SHOOTER_POWER);
            rightShooter.setPower(SHOOTER_POWER);
        } else {
            leftShooter.setPower(0);
            rightShooter.setPower(0);
        }

        dashboard.displayPrintf(1, LABEL_WIDTH, "digital input: ", "%b", inputPin);
        dashboard.displayPrintf(2, LABEL_WIDTH, "servo position: ", "%1.3f", triggerServo.getPosition());
        dashboard.displayPrintf(4, LABEL_WIDTH, "rotation: ", "%.2f", rotation);
        dashboard.displayPrintf(5, LABEL_WIDTH, "gamepad left stick direction true: ", "%.2f", gamepad.getLeftStickDirectionDegrees(true));
        dashboard.displayPrintf(6, LABEL_WIDTH, "gamepad left stick direction false: ", "%.2f", gamepad.getLeftStickDirectionDegrees(false));
        dashboard.displayPrintf(7, LABEL_WIDTH, "gamepad right stick x: ", "%1.2f", gamepad.getRightStickX());
        dashboard.displayPrintf(8, LABEL_WIDTH, "gamepad right stick y: ", "%1.2f", gamepad.getRightStickY());
        dashboard.displayPrintf(9, LABEL_WIDTH, "y inverted: ", "%b", setYInverted);
        dashboard.displayPrintf(10, LABEL_WIDTH, "x inverted: ", "%b", setXInverted);
        dashboard.displayPrintf(11, LABEL_WIDTH, "fixedOnTarget: ", "%b", fixedOnTarget);
    }

    private ElapsedTime getButtonElapsedTime(int button){
        if(toggleTimeTracker.containsKey(button))
            return toggleTimeTracker.get(button);
        else {
            toggleTimeTracker.put(button, new ElapsedTime());
            return toggleTimeTracker.get(button);
        }
    }

    @Override
    public void gamepadButtonEvent(SWGamePad gamepad, int button, boolean pressed)
    {
        if (gamepad == this.gamepad)
        {
            switch (button)
            {
                case SWGamePad.GAMEPAD_A:
                    //driveBase.enableGyroAssist(0.00001, 0.05);
                    break;

                case SWGamePad.GAMEPAD_Y:
                    DbgLog.msg("PRESSED VALUE IS value %b", pressed);
                    DbgLog.msg("PRESSED VALUE IS value %0.2f", getButtonElapsedTime(button).seconds());
                    if(pressed && getButtonElapsedTime(button).seconds() > 1){
                        DbgLog.msg("inverting value");
                        fixedOnTarget = !fixedOnTarget;
                        getButtonElapsedTime(button).reset();
                    }
                    break;

                case SWGamePad.GAMEPAD_X:
                    break;

                case SWGamePad.GAMEPAD_B:
                    //driveBase.disableGyroAssist();
                    break;

                case SWGamePad.GAMEPAD_RBUMPER:
                    break;

                case SWGamePad.GAMEPAD_DPAD_DOWN:
                    break;

                case SWGamePad.GAMEPAD_DPAD_UP:
                    if(pressed && getButtonElapsedTime(button).seconds() > 1){
                        if(!checkToggleTime(button))
                            break;

                        setYInverted = !setYInverted;
                        getButtonElapsedTime(button).reset();
                    }
                    break;

                case SWGamePad.GAMEPAD_DPAD_LEFT:
                    break;

                case SWGamePad.GAMEPAD_DPAD_RIGHT:
                    if(pressed && getButtonElapsedTime(button).seconds() > 1){
                        if(!checkToggleTime(button))
                            break;

                        setXInverted = !setXInverted;
                        getButtonElapsedTime(button).reset();
                    }
                    break;
            }
        }
    }

    private boolean checkToggleTime(int button){
        return (getButtonElapsedTime(button).seconds() > 0.75);
    }

    //THE THREAD THAT WILL RUN THE TRIGGER
    @Override
    public void run() {
        while (OP_MODE_IS_ACTIVE){
            inputPin = digIn.getState();
            if(!gamepad1.left_bumper) {
                SHOOTER_MOTORS_REVERSE = false;
                if (inputPin && gamepad1.right_bumper) {

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
                    SHOOTER_MOTORS_ACTIVE = false;
                    SHOOTER_MOTORS_IS_READY = false;

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
