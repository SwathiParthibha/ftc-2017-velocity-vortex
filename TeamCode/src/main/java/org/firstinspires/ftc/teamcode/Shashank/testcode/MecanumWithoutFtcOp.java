package org.firstinspires.ftc.teamcode.Shashank.testcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import ftclib.FtcDcMotor;
import hallib.HalDashboard;
import swlib.SWGamePad;
import swlib.SWMRGyro;
import trclib.TrcDriveBase;
import trclib.TrcGyro;
import trclib.TrcRobot;
import trclib.TrcSensor;
import trclib.TrcTaskMgr;

/**
 * Created by spmeg on 3/25/2017.
 */
@TeleOp(name = "MecanumWithoutFtcOp", group = "WHATEVER")
public class MecanumWithoutFtcOp extends OpMode implements SWGamePad.ButtonHandler{
    private FtcDcMotor leftFrontMotor;
    private FtcDcMotor leftRearMotor;
    private FtcDcMotor rightFrontMotor;
    private FtcDcMotor rightRearMotor;
    private TrcDriveBase driveBase = null;
    private SWGamePad gamepad;
    private boolean fixedOnTarget = false;
    private SWMRGyro gyro = null;
    private HalDashboard dashboard = null;
    private final int LABEL_WIDTH = 200;
    private boolean setYInverted = true;
    private boolean setXInverted = false;
    private TrcTaskMgr taskMgr = new TrcTaskMgr();

    @Override
    public void init_loop() {
        super.init_loop();
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
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK, TrcRobot.RunMode.TELEOP_MODE);
    }

    @Override
    public void init() {
        dashboard = HalDashboard.createInstance(this.telemetry);
        DbgLog.msg("> INIT" + "STARTED INIT");

        DbgLog.msg("> INIT" + "STARTED OBJECT CREATION OF GYRO");
        gyro = new SWMRGyro(hardwareMap, "gyro", null);
        gyro.calibrate();
        DbgLog.msg("> INIT" + "FINISHED OBJECT CREATION OF GYRO");

        DbgLog.msg("> INIT" + "GENERATING MOTORS");
        leftFrontMotor = new FtcDcMotor(this.hardwareMap, "motor_1", null, null);
        leftRearMotor = new FtcDcMotor(this.hardwareMap, "motor_3", null, null);
        rightFrontMotor = new FtcDcMotor(this.hardwareMap, "motor_4", null, null);
        rightRearMotor = new FtcDcMotor(this.hardwareMap, "motor_2", null, null);
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
        driveBase = new TrcDriveBase(
                leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, gyro);
        /*driveBase = new TrcDriveBase(
                leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);*/
        DbgLog.msg("> INIT" + "FINISHED INITING TRCDRIVEBASE");

        DbgLog.msg("> INIT" + "CREATING GAMEPAD");
        gamepad = new SWGamePad("driver gamepad", gamepad1, this);
        gamepad1.setJoystickDeadzone(0.05F);
        gamepad.enableDebug(true);
        DbgLog.msg("> INIT" + "FINISHED CREATING GAMEPAD");

        DbgLog.msg("> INIT" + "WAITING FOR GYRO TO CALIBRATE");
        // make sure the gyro is calibrated.
        /*while (gyro.isCalibrating())  {
            sleep(50);
        }*/
        DbgLog.msg("> INIT" + "FINISHED WAITING FOR GYRO TO CALIBRATE");

        DbgLog.msg("> INIT" + "ENABLING GYRO");
        gyro.setEnabled(true);
        DbgLog.msg("> INIT" + "FINISHED ENABLING GYRO");

        //DbgLog.msg("> INIT" + "DISABLING GYRO ASSIST");
        //driveBase.disableGyroAssist();
        //DbgLog.msg("> INIT" + "FINISHED DISABLING GYRO ASSIST");

        DbgLog.msg("> INIT" + "ENABLING GYRO ASSIST");
        //driveBase.enableGyroAssist(0.00001, 0.05);
        DbgLog.msg("> INIT" + "FINISHED ENABLING GYRO ASSIST");

        DbgLog.msg("> INIT" + "FINISHED INIT");
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_down)
            setYInverted = false;
        if(gamepad1.dpad_up)
            setYInverted = true;
        if(gamepad1.dpad_left)
            setXInverted = false;
        if(gamepad1.dpad_right)
            setXInverted = true;
        if(gamepad1.a)
            driveBase.enableGyroAssist(0.00001, 0.05);
        if(gamepad1.b)
            driveBase.disableGyroAssist();
        if(gamepad1.x)
            fixedOnTarget = true;
        if(gamepad1.y)
            fixedOnTarget = false;

        double x = 0;
        double y = 0;
        x = gamepad.getLeftStickX(false) * (setXInverted ? -1:1);;
        y = gamepad.getLeftStickY(false);

        gamepad.setYInverted(setYInverted);
        if(gyro.getRawZData(TrcGyro.DataType.HEADING).value > 97
                && gyro.getRawZData(TrcGyro.DataType.HEADING).value < 278){
            y = y * -1;
            x = x * -1;
        }

        double rotation = gamepad.getRightStickX();
        TrcSensor.SensorData<Double> sensorData = gyro.getZHeading();
        driveBase.mecanumDrive_Cartesian(x, y, rotation, false,
                fixedOnTarget? sensorData.value: 0.0);
        dashboard.displayPrintf(0, LABEL_WIDTH, "x: ", "%.2f", x);
        dashboard.displayPrintf(1, LABEL_WIDTH, "y: ", "%.2f", y);
        dashboard.displayPrintf(2, LABEL_WIDTH, "rotation: ", "%.2f", rotation);
        dashboard.displayPrintf(3, LABEL_WIDTH, "gyro trc: ", "%.2f", gyro.getRawZData(TrcGyro.DataType.HEADING).value);
        dashboard.displayPrintf(4, LABEL_WIDTH, "gyro trc rate: ", "%.2f", gyro.getRawZData(TrcGyro.DataType.ROTATION_RATE).value);
        dashboard.displayPrintf(5, LABEL_WIDTH, "gamepad left stick direction: ", "%.2f", gamepad.getLeftStickDirectionDegrees(true));
        dashboard.displayPrintf(6, LABEL_WIDTH, "gamepad left stick " +
                "magnitude: ", "%.2f", gamepad.getLeftStickMagnitude());
        dashboard.displayPrintf(7, LABEL_WIDTH, "gamepad right stick x: ", "%.2f", gamepad.getRightStickX());
        dashboard.displayPrintf(8, LABEL_WIDTH, "gamepad right stick x * -1: ", "%.2f", gamepad.getRightStickX()*-1);
        dashboard.displayPrintf(9, LABEL_WIDTH, "y inverted: ", "%b", setYInverted);
        dashboard.displayPrintf(10, LABEL_WIDTH, "x inverted: ", "%b", setXInverted);
        dashboard.displayPrintf(11, LABEL_WIDTH, "fixedOnTarget: ", "%b", fixedOnTarget);
        dashboard.displayPrintf(12, LABEL_WIDTH, "front left motor power: ", "%.2f", leftFrontMotor.getPower());
        dashboard.displayPrintf(13, LABEL_WIDTH, "front right motor power: ", "%.2f", rightFrontMotor.getPower());
        dashboard.displayPrintf(14, LABEL_WIDTH, "back lwft motor power: ", "%.2f", leftRearMotor.getPower());
        dashboard.displayPrintf(15, LABEL_WIDTH, "back right motor power: ", "%.2f", rightRearMotor.getPower());
    }

    @Override
    public void gamepadButtonEvent(SWGamePad gamepad, int button, boolean pressed)
    {
        if (gamepad == this.gamepad)
        {
            switch (button)
            {
                case SWGamePad.GAMEPAD_A:
                    break;

                case SWGamePad.GAMEPAD_Y:
                    break;

                case SWGamePad.GAMEPAD_X:
                    break;

                case SWGamePad.GAMEPAD_B:
                    break;

                case SWGamePad.GAMEPAD_RBUMPER:
                    //fixedOnTarget = pressed;
                    break;
                case SWGamePad.GAMEPAD_DPAD_DOWN:
                    setYInverted = true;
                    break;
                case SWGamePad.GAMEPAD_DPAD_UP:
                    setYInverted = false;
                    break;
                case SWGamePad.GAMEPAD_DPAD_LEFT:
                    setXInverted = false;
                    break;
                case SWGamePad.GAMEPAD_DPAD_RIGHT:
                    setXInverted = true;
                    break;
            }
        }
    }

}
