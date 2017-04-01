package org.firstinspires.ftc.teamcode.Shashank.testcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import hallib.HalDashboard;
import swlib.SWGamePad;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;

/**
 * Created by spmeg on 3/31/2017.
 */
@TeleOp(name = "TestTeleop", group = "Whatever")
public class TestTeleop extends OpMode implements SWGamePad.ButtonHandler{
    private SWGamePad gamepad;
    private HalDashboard dashboard = null;
    private final int LABEL_WIDTH = 200;
    private boolean setYInverted = false;
    private boolean setXInverted = false;
    private TrcTaskMgr taskMgr = new TrcTaskMgr();

    @Override
    public void init_loop() {
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK, TrcRobot.RunMode.TELEOP_MODE);
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.PREPERIODIC_TASK, TrcRobot.RunMode.TELEOP_MODE);
        super.init_loop();
    }

    @Override
    public void start() {
        DbgLog.msg("> START" + "CREATING GAMEPAD");
        gamepad = new SWGamePad("driver gamepad", gamepad1, this);
        gamepad1.setJoystickDeadzone(0.05F);
        DbgLog.msg("> START" + "FINISHED CREATING GAMEPAD");
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.START_TASK, TrcRobot.RunMode.TELEOP_MODE);
        super.start();
    }

    @Override
    public void stop() {
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.STOP_TASK, TrcRobot.RunMode.TELEOP_MODE);
        super.stop();
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
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK, TrcRobot.RunMode.TELEOP_MODE);
        super.postLoop();
    }

    @Override
    public void init() {
        dashboard = HalDashboard.createInstance(this.telemetry);
        DbgLog.msg("> INIT" + "STARTED INIT");

        DbgLog.msg("> INIT" + "CREATING GAMEPAD");
        gamepad = new SWGamePad("driver gamepad", gamepad1, this);
        gamepad1.setJoystickDeadzone(0.05F);
        DbgLog.msg("> INIT" + "FINISHED CREATING GAMEPAD");

        DbgLog.msg("> INIT" + "FINISHED INIT");
    }

    @Override
    public void loop() {
        gamepad.setYInverted(setYInverted);
        dashboard.displayPrintf(6, LABEL_WIDTH, "gamepad left stick direction: ", "%.2f", gamepad.getLeftStickDirectionDegrees(true));
        dashboard.displayPrintf(7, LABEL_WIDTH, "gamepad left stick " +
                "magnitude: ", "%.2f", gamepad.getLeftStickMagnitude());
        dashboard.displayPrintf(8, LABEL_WIDTH, "gamepad right stick x: ", "%.2f", gamepad.getRightStickX());
        dashboard.displayPrintf(9, LABEL_WIDTH, "gamepad right stick x * -1: ", "%.2f", gamepad.getRightStickX()*-1);
        dashboard.displayPrintf(10, LABEL_WIDTH, "y inverted: ", "%b", setYInverted);
        dashboard.displayPrintf(11, LABEL_WIDTH, "x inverted: ", "%b", setXInverted);
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
