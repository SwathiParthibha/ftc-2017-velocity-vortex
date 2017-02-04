package org.firstinspires.ftc.teamcode.Shashank.testcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import ftc.electronvolts.statemachine.StateMachine;
import ftc.electronvolts.statemachine.StateMachineBuilder;
import ftc.electronvolts.statemachine.StateName;

/**
 * Created by spmeg on 1/20/2017.
 */
@Autonomous(name = "TestStateMachineOp", group = "TestOpmodes")
@Disabled
public class TestStateMachineOp extends OpMode {
    private StateMachineBuilder b = null;
    private StateMachine stateMachine = null;

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    enum S implements StateName {
        TEST,
        DRIVE,
        WAIT,
        STOP
    };

    @Override
    public void init() {
        leftMotor   = hardwareMap.dcMotor.get("l");
        rightMotor  = hardwareMap.dcMotor.get("r");

        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        TestStateMachineBuilder b = new TestStateMachineBuilder(S.TEST);
        b.addTelem(S.TEST, S.DRIVE, this.telemetry);

        b.addDrive(S.DRIVE, S.WAIT, leftMotor, rightMotor);
        b.addTelem(S.WAIT, S.DRIVE, this.telemetry);
        b.addDrive(S.DRIVE, S.WAIT, leftMotor, rightMotor);
        //add a wait state named WAIT for 1000 milliseconds, then go to the state named STOP
        b.addWait(S.WAIT, S.STOP, 1000);

        //add a stop state named STOP
        b.addStop(S.STOP);

        stateMachine = b.build();
    }

    @Override
    public void loop() {
        stateMachine.act();

        //use this for the FTC app:
        telemetry.addData("State", stateMachine.getCurrentStateName());

        //use this for other usages:
        System.out.println("State: " + stateMachine.getCurrentStateName());

        telemetry.update();
    }
}
