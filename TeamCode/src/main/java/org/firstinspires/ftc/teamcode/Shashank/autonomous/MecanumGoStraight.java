package org.firstinspires.ftc.teamcode.Shashank.autonomous;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
import trclib.TrcUtil;

/**
 * Created by spmeg on 3/31/2017.
 */
@Autonomous(name = "MecanumTest", group = "Autonomous")
@Disabled
public class MecanumGoStraight extends OpMode {
    private FtcDcMotor leftFrontMotor;
    private FtcDcMotor leftRearMotor;
    private FtcDcMotor rightFrontMotor;
    private FtcDcMotor rightRearMotor;
    private TrcDriveBase driveBase = null;
    private boolean fixedOnTarget = true;
    private SWMRGyro gyro = null;
    private HalDashboard dashboard = null;
    private final int LABEL_WIDTH = 200;
    private boolean setYInverted = true;
    private boolean setXInverted = false;
    private TrcTaskMgr taskMgr = new TrcTaskMgr();
    private double startTime = 0.0;

    private enum State{
        GO_STRAIGHT,
        STOP
    }

    State state = State.GO_STRAIGHT;

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

        DbgLog.msg("> INIT" + "WAITING FOR GYRO TO CALIBRATE");
        // make sure the gyro is calibrated.
        while (gyro.isCalibrating()) {
            TrcUtil.sleep(50);
        }
        DbgLog.msg("> INIT" + "FINISHED WAITING FOR GYRO TO CALIBRATE");

        DbgLog.msg("> INIT" + "ENABLING GYRO");
        gyro.setEnabled(true);
        DbgLog.msg("> INIT" + "FINISHED ENABLING GYRO");

        //DbgLog.msg("> INIT" + "DISABLING GYRO ASSIST");
        //driveBase.disableGyroAssist();
        //DbgLog.msg("> INIT" + "FINISHED DISABLING GYRO ASSIST");

        DbgLog.msg("> INIT" + "ENABLING GYRO ASSIST");
        driveBase.enableGyroAssist(0.00001, 0.05);
        DbgLog.msg("> INIT" + "FINISHED ENABLING GYRO ASSIST");

        startTime = getRuntime();

        DbgLog.msg("> INIT" + "FINISHED INIT");
    }

    @Override
    public void loop() {

        double x = 1;
        double y = 1;

        switch (state){
            case GO_STRAIGHT:
                if(getRuntime() - startTime > 2.5)
                    state = State.STOP;
                x = 0;
                y = 1;
                break;
            case STOP:
                x = 0;
                y = 0;
                break;
        }

        if (gyro.getRawZData(TrcGyro.DataType.HEADING).value > 90 && gyro.getRawZData(TrcGyro.DataType.HEADING).value < 270) {
            y = y * -1;
            x = x * -1;
        }

        double rotation = 0;
        TrcSensor.SensorData<Double> sensorData = gyro.getZHeading();

        driveBase.mecanumDrive_Cartesian(x, y, rotation, false,
                fixedOnTarget ? sensorData.value : 0.0);

        dashboard.displayPrintf(1, LABEL_WIDTH, "x: ", "%.2f", x);
        dashboard.displayPrintf(2, LABEL_WIDTH, "y: ", "%.2f", y);
        dashboard.displayPrintf(3, LABEL_WIDTH, "rotation: ", "%.2f", rotation);
        dashboard.displayPrintf(4, LABEL_WIDTH, "gyro trc: ", "%.2f", gyro.getRawZData(TrcGyro.DataType.HEADING).value);
        dashboard.displayPrintf(5, LABEL_WIDTH, "gyro trc rate: ", "%.2f", gyro.getRawZData(TrcGyro.DataType.ROTATION_RATE).value);
        dashboard.displayPrintf(6, LABEL_WIDTH, "y inverted: ", "%b", setYInverted);
        dashboard.displayPrintf(7, LABEL_WIDTH, "x inverted: ", "%b", setXInverted);
    }

}