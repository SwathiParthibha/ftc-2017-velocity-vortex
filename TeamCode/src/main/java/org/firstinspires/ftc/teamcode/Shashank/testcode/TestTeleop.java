package org.firstinspires.ftc.teamcode.Shashank.testcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

import ftclib.FtcDcMotor;
import hallib.HalDashboard;
import swlib.SWGamePad;
import swlib.SWIMUGyro;
import trclib.TrcDriveBase;
import trclib.TrcGyro;
import trclib.TrcRobot;
import trclib.TrcSensor;
import trclib.TrcTaskMgr;

/**
 * Created by spmeg on 3/31/2017.
 */
@TeleOp(name = "TestTeleop", group = "Whatever")
public class TestTeleop extends OpMode implements SWGamePad.ButtonHandler{
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

        DbgLog.msg("> INIT" + "CREATING GAMEPAD");
        gamepad = new SWGamePad("driver gamepad", gamepad1, 0.05F, this);
        gamepad.enableDebug(true);
        DbgLog.msg("> INIT" + "FINISHED CREATING GAMEPAD");

        DbgLog.msg("> INIT" + "FINISHED INIT");
    }

    @Override
    public void loop() {
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.PREPERIODIC_TASK, TrcRobot.RunMode.TELEOP_MODE);
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK, TrcRobot.RunMode.TELEOP_MODE);
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.PREPERIODIC_TASK, TrcRobot.RunMode.AUTO_MODE);
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK, TrcRobot.RunMode.AUTO_MODE);

        double x = 0;
        double y = 0;
        x = gamepad.getLeftStickX(false) * (setXInverted ? -1:1);;
        y = gamepad.getLeftStickY(false);

        gamepad.setYInverted(setYInverted);

        double rotation = gamepad.getRightStickX();
        dashboard.displayPrintf(0, LABEL_WIDTH, "x: ", "%.2f", x);
        dashboard.displayPrintf(1, LABEL_WIDTH, "y: ", "%.2f", y);
        dashboard.displayPrintf(2, LABEL_WIDTH, "rotation: ", "%.2f", rotation);
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

}
