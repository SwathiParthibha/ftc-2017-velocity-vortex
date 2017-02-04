package org.firstinspires.ftc.teamcode.Shashank.utils;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by spmeg on 1/21/2017.
 */

public class BugEyesBot implements BasicBot {
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor shooterLeft = null;
    public DcMotor shooterRight = null;
    public DcMotor sweeper = null;
    public DcMotor scooperChain = null;

    public LightSensor lightSensor;      // Primary LEGO Light sensor
    public ColorSensor leftColorSensor = null;
    public ColorSensor rightColorSensor = null;
    public BNO055IMU imu;
    I2cDeviceSynchImpl rangeSensor;
    I2cDeviceSynchImpl sideRangeSensor;

    public Orientation angles;

    public double angleZ = 0;

    static final double APPROACH_SPEED = 0.5;
    double WHEEL_SIZE_IN = 4;
    public int ROTATION = 1220; // # of ticks for 40-1 gear ratio
    static final double     DRIVE_GEAR_REDUCTION    = 1.5 ;     // This is < 1.0 if geared UP
    double GEAR_RATIO = 40;
    double     COUNTS_PER_INCH         = (ROTATION * DRIVE_GEAR_REDUCTION) /
            (WHEEL_SIZE_IN * Math.PI) * (40 / GEAR_RATIO);
    byte[] rangeSensorCache;
    byte[] sideRangeSensorCache;
    I2cDevice rangeA;
    I2cDevice rangeB;

    HardwareMap hwMap = null;
    Telemetry telemetry;

    private boolean continueBot = true;

    public BugEyesBot(HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
    }

    @Override
    public void init() {
        leftMotor   = hwMap.dcMotor.get("l");
        rightMotor  = hwMap.dcMotor.get("r");

        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        lightSensor = hwMap.lightSensor.get("light sensor");
        rangeA = hwMap.i2cDevice.get("range sensor");// Primary LEGO Light Sensor
        rangeSensor = new I2cDeviceSynchImpl(rangeA, I2cAddr.create8bit(0x2a), false);
        rangeA = hwMap.i2cDevice.get("r side range");// Primary LEGO Light Sensor
        sideRangeSensor = new I2cDeviceSynchImpl(rangeA, I2cAddr.create8bit(0x28), false);
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        rangeSensor.engage();
        sideRangeSensor.engage();
    }

    @Override
    public void drive(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    @Override
    public void turn(int turnAngle)
    {
        //robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        angleZ = IMUheading();

        double angDiff = turnAngle-angleZ; //positive: turn left
        //if (Math.abs(angDiff) > 180) angDiff = angDiff % 180;

        if (angDiff < 0) { //turns right
            leftMotor.setPower(APPROACH_SPEED * .6 );
            rightMotor.setPower(-APPROACH_SPEED * .6);

            while ((angDiff < 0) && continueBot) {

                angleZ = IMUheading();
                angDiff = turnAngle-angleZ;

                if (Math.abs(angDiff) < 90) {
                    leftMotor.setPower(APPROACH_SPEED * .2);
                    rightMotor.setPower(-APPROACH_SPEED * .2);
                }
                else if (Math.abs(angDiff) < 45) {
                    leftMotor.setPower(APPROACH_SPEED * .05);
                    rightMotor.setPower(-APPROACH_SPEED * .05);
                }

                telemetry.addData("Angle", angleZ);
                telemetry.update();
            }
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }

        else if (angDiff > 0) { //turns left
            leftMotor.setPower(-APPROACH_SPEED);
            rightMotor.setPower(APPROACH_SPEED);

            while ((angDiff > 0) && continueBot) {

                angleZ = IMUheading();
                angDiff = turnAngle-angleZ;

                if (Math.abs(angDiff) < 90) {
                    leftMotor.setPower(-APPROACH_SPEED * .2);
                    rightMotor.setPower(APPROACH_SPEED * .2);
                }
                else if (Math.abs(angDiff) < 45) {
                    leftMotor.setPower(-APPROACH_SPEED * .05);
                    rightMotor.setPower(APPROACH_SPEED * .05);
                }

                telemetry.addData("Angle", angleZ);
                telemetry.update();

            }
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
        //robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    double IMUheading() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    @Override
    public void driveToPos(int targetPos) {

    }

    @Override
    public void driveTimeout(int time) {

    }
}
