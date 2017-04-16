package org.firstinspires.ftc.teamcode.Shashank.statemachine.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Shashank.statemachine.MecanumStateMachineBuilder;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.states.endconditions.TimerEndConditions;

import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateName;
import ftclib.FtcDcMotor;
import hallib.HalDashboard;
import swlib.SWIMUGyro;
import trclib.TrcDriveBase;
import trclib.TrcTaskMgr;

/**
 * Created by spmeg on 4/14/2017.
 */
@Autonomous(name = "GoSraightStateMachineOp", group = "auto")
public class GoSraightStateMachineOp extends OpMode {
    private StateMachine stateMachine = null;
    private FtcDcMotor leftFrontMotor;
    private FtcDcMotor leftRearMotor;
    private FtcDcMotor rightFrontMotor;
    private FtcDcMotor rightRearMotor;
    private TrcDriveBase driveBase = null;
    private SWIMUGyro gyro = null;
    private HalDashboard dashboard = null;
    private final int LABEL_WIDTH = 200;
    private TrcTaskMgr taskMgr = new TrcTaskMgr();

    public enum StateNames implements StateName{
        WAIT,
        GO_FORWARD,
        STOP
    }

    StateNames stateNames = StateNames.WAIT;

    @Override
    public void init() {
        dashboard = HalDashboard.createInstance(this.telemetry);
        DbgLog.msg("> INIT" + "STARTED INIT");

        leftFrontMotor = new FtcDcMotor(this.hardwareMap, "leftFront", null, null);
        leftRearMotor = new FtcDcMotor(this.hardwareMap, "leftRear", null, null);
        rightFrontMotor = new FtcDcMotor(this.hardwareMap, "rightFront", null, null);
        rightRearMotor = new FtcDcMotor(this.hardwareMap, "rightRear", null, null);

        leftFrontMotor.setInverted(true);
        leftRearMotor.setInverted(true);

        leftFrontMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveBase = new TrcDriveBase(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, null);

        DbgLog.msg("> INIT" + "FINISHED INIT");
    }

    @Override
    public void start() {
        super.stop();
        DbgLog.msg("IN START");

        MecanumStateMachineBuilder mecanumStateMachineBuilder = new MecanumStateMachineBuilder(StateNames.WAIT, driveBase);

        mecanumStateMachineBuilder.addWait(StateNames.WAIT, StateNames.GO_FORWARD, 1000);
        mecanumStateMachineBuilder.addDrivePolar(StateNames.GO_FORWARD, driveBase, 1.0, 0, 0.0, StateNames.STOP, TimerEndConditions.timerSeconds(5));
        mecanumStateMachineBuilder.addStop(StateNames.STOP);

        stateMachine = mecanumStateMachineBuilder.build();

        DbgLog.msg("FINISHED START");
    }

    @Override
    public void loop() {
        stateMachine.act();
        dashboard.displayPrintf(1, LABEL_WIDTH, "CURRENT STATE: ", "%s", stateMachine.getCurrentStateName());
    }
}
