package org.firstinspires.ftc.teamcode.Shashank.testcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftclib.FtcDcMotor;
import hallib.HalDashboard;
import swlib.SWGamePad;
import swlib.SWMRGyro;
import trclib.TrcDriveBase;
import trclib.TrcGyro;
import trclib.TrcTaskMgr;

/**
 * Created by spmeg on 3/28/2017.
 */
@TeleOp(name = "MRTRCTestOp", group = "TestCodeShooter")
public class MRTRCTestOp extends OpMode {
    private SWMRGyro gyro = null;
    private HalDashboard dashboard = null;
    private final int LABEL_WIDTH = 200;

    @Override
    public void init() {new TrcTaskMgr();
        dashboard = HalDashboard.createInstance(this.telemetry);

        gyro = new SWMRGyro(hardwareMap, "gyro", null);
        gyro.calibrate();
    }

    @Override
    public void loop() {
        dashboard.displayPrintf(1, LABEL_WIDTH, "gyro.getZHeading().value: ", "%.2f", gyro.getZHeading().value);
        dashboard.displayPrintf(2, LABEL_WIDTH, "((ModernRoboticsI2cGyro)hardwareMap.get(\"gyro\")).getHeading(): ", "%d", ((ModernRoboticsI2cGyro)hardwareMap.get("gyro")).getHeading());
        dashboard.displayPrintf(3, LABEL_WIDTH, "gyro.getRawZData(TrcGyro.DataType.ROTATION_RATE): ", "%.2f", gyro.getRawZData(TrcGyro.DataType.ROTATION_RATE).value);
        dashboard.displayPrintf(4, LABEL_WIDTH, "gyro.getRawZData(TrcGyro.DataType.HEADING): ", "%.2f", gyro.getRawZData(TrcGyro.DataType.HEADING).value);
    }
}
