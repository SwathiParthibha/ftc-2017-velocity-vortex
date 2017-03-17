package org.firstinspires.ftc.teamcode.Shashank.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by spmeg on 10/22/2016.
 */
@TeleOp(name = "ThreadedTeleop", group = "Tests")
@Disabled
public class ThreadedTeleop extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private ThreadMsger threadMsger = new ThreadMsger();
    private JoyStickThread joyStickThread = null;
    private MotorThread motorThread = null;

    @Override
    public void init() {
        leftMotor = this.hardwareMap.dcMotor.get("l");
        rightMotor = this.hardwareMap.dcMotor.get("r");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.log().add("Finished init of motors");

        Thread thread = new Thread();
        thread.start();

        telemetry.log().add("Init motor thread");
        motorThread = new MotorThread(threadMsger, leftMotor, rightMotor);
        sleep(100);
        telemetry.log().add("Starting motor thread");
        //motorThread.start();
        sleep(100);
        telemetry.log().add("Started motor thread");
        sleep(100);

        telemetry.log().add("Init joystick thread");
        joyStickThread = new JoyStickThread(this.gamepad1, threadMsger);
        sleep(100);

        telemetry.log().add("Starting joystick thread");
        //joyStickThread.start();
        sleep(100);
        telemetry.log().add("Started joystick thread");
        sleep(100);
    }

    @Override
    public void loop() {
        double left = -threadMsger.getLeftJoystickValue();
        double right = -threadMsger.getRightJoystickValue();

        telemetry.addData("left joystick",  "%.2f", left);
        telemetry.addData("right joystick", "%.2f", right);
        telemetry.addData("test Value, true if successful", threadMsger.getTestValue());
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
        threadMsger.setRequestStop(true);
        joyStickThread.requestStop();
        motorThread.requestStop();
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
