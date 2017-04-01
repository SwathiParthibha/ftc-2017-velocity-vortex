package org.firstinspires.ftc.teamcode.Shashank.testcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import ftclib.FtcDcMotor;
import ftclib.FtcGamepad;
import ftclib.FtcMRGyro;
import ftclib.FtcOpMode;
import trclib.TrcDriveBase;

/**
 * Created by spmeg on 3/24/2017.
 */
@TeleOp(name = "MecanumTestFtcTeleOp", group = "Whatever")
public class MecanumTestFtcTeleOp extends FtcOpMode implements FtcGamepad.ButtonHandler{
    private DcMotor dcMotor1, dcMotor2, dcMotor3, dcMotor4;
    private FtcDcMotor leftFrontMotor;
    private FtcDcMotor leftRearMotor;
    private FtcDcMotor rightFrontMotor;
    private FtcDcMotor rightRearMotor;
    private TrcDriveBase driveBase = null;
    private FtcGamepad gamepad;
    private boolean fixedOnTarget = false;
    private FtcMRGyro gyro = null;

    @Override
    public void initRobot() {
        dcMotor1 = this.hardwareMap.dcMotor.get("motor_1");
        dcMotor2 = this.hardwareMap.dcMotor.get("motor_4");
        dcMotor3 = this.hardwareMap.dcMotor.get("motor_3");
        dcMotor4 = this.hardwareMap.dcMotor.get("motor_2");

        gyro = new FtcMRGyro("gyro");
        gyro.calibrate();

        leftFrontMotor = new FtcDcMotor(this.hardwareMap, "motor_1", null, null);
        leftRearMotor = new FtcDcMotor(this.hardwareMap, "motor_3", null, null);
        rightFrontMotor = new FtcDcMotor(this.hardwareMap, "motor_4", null, null);
        rightRearMotor = new FtcDcMotor(this.hardwareMap, "motor_2", null, null);

        leftFrontMotor.setInverted(true);
        leftRearMotor.setInverted(true);

        driveBase = new TrcDriveBase(
                leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, gyro);

        gamepad = new FtcGamepad("Gamepad", gamepad1, this);
        gamepad.setYInverted(false);

        // make sure the gyro is calibrated.
        while (gyro.isCalibrating())  {
            sleep(50);
        }

        gyro.setEnabled(true);
        driveBase.disableGyroAssist();
    }

    @Override
    public void runPeriodic(double elapsedTime) {
        double x = gamepad.getLeftStickX(true);
        double y = gamepad.getRightStickY(true);
        double rotation = gamepad.getRightTrigger(true);
        if (rotation == 0.0)
        {
            rotation = -gamepad.getLeftTrigger(true);
        }
        driveBase.mecanumDrive_Cartesian(x, y, rotation, false,
                fixedOnTarget? (Double)gyro.getZHeading().value: 0.0);
    }

    @Override
    public void stopMode()
    {
        gyro.setEnabled(false);
    }   //stopMode

    @Override
    public void gamepadButtonEvent(FtcGamepad gamepad, int button, boolean pressed)
    {
        if (gamepad == this.gamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    break;

                case FtcGamepad.GAMEPAD_X:
                    break;

                case FtcGamepad.GAMEPAD_B:
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:
                    fixedOnTarget = pressed;
                    break;
            }
        }
    }
}
