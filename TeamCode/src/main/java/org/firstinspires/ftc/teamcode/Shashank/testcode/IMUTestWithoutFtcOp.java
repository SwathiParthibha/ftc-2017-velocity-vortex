package org.firstinspires.ftc.teamcode.Shashank.testcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import ftclib.FtcDcMotor;
import ftclib.FtcGamepad;
import hallib.HalDashboard;
import swlib.SWIMUGyro;
import swlib.SWMRGyro;
import trclib.TrcDriveBase;
import trclib.TrcGyro;
import trclib.TrcSensor;
import trclib.TrcTaskMgr;

/**
 * Created by spmeg on 3/25/2017.
 */
@TeleOp(name = "IMUTestWithoutFtcOp", group = "WHATEVER")
public class IMUTestWithoutFtcOp extends OpMode{
    private HalDashboard dashboard = null;
    private final int LABEL_WIDTH = 200;
    private SWIMUGyro swimuGyro = null;

    @Override
    public void init() {
        new TrcTaskMgr();
        dashboard = HalDashboard.createInstance(this.telemetry);
        DbgLog.msg("> INIT" + "STARTED INIT");

        swimuGyro = new SWIMUGyro(hardwareMap, "imu", null);
        swimuGyro.calibrate();

        DbgLog.msg("> INIT" + "FINISHED INIT");
    }

    @Override
    public void loop() {
        DbgLog.msg("> LOOP" + "START");
        dashboard.displayPrintf(1, LABEL_WIDTH, "imu z trc heading: ", "%.2f", swimuGyro.getZHeading().value);
        dashboard.displayPrintf(2, LABEL_WIDTH, "imu z trc raw: ", "%.2f", swimuGyro.getRawZData(TrcGyro.DataType.ROTATION_RATE).value);
        dashboard.displayPrintf(3, LABEL_WIDTH, "imu trc calib: ", "%b", swimuGyro.isCalibrating());
        dashboard.displayPrintf(4, LABEL_WIDTH, "imu z: ", "%.2f", hardwareMap.get(BNO055IMU.class, "imu").getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).secondAngle);
        DbgLog.msg("> LOOP" + "END");
    }

}
