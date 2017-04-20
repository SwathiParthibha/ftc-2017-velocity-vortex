/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.Mrinali;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.AllianceColor;

import ftclib.FtcDcMotor;
import ftclib.FtcOpMode;
import trclib.TrcDriveBase;

/**
 * This file illustrates the concept of driving up to a line and then stopping.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code shows using two different light sensors:
 *   The Primary sensor shown in this code is a legacy NXT Light sensor (called "light sensor")
 *   Alternative "commented out" code uses a MR Optical Distance Sensor (called "sensor_ods")
 *   instead of the LEGO sensor.  Chose to use one sensor or the other.
 *
 *   Setting the correct WHITE_THRESHOLD value is key to stopping correctly.
 *   This should be set half way between the light and dark values.
 *   These values can be read on the screen once the OpMode has been INIT, but before it is STARTED.
 *   Move the senso on asnd off the white line and not the min and max readings.
 *   Edit this code to make WHITE_THRESHOLD half way between the min and max.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

public class MecanumAuto {

    //To change red to blue: negative angles, color sensors sense blue, right side range sensor

    /* Declare OpMode members. */
    //HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    FtcOpMode opMode;
    Telemetry telemetry;
    AllianceColor color;
    public TrcDriveBaseIMU driveBase;
    public FtcDcMotor frontLeftMotor;
    public FtcDcMotor frontRightMotor;
    public FtcDcMotor backLeftMotor;
    public FtcDcMotor backRightMotor;
    public BNO055IMU imu;
    Orientation angles;
    double angleZ;
    public ColorSensor leftColorSensor;
    public ColorSensor rightColorSensor;
    public ColorSensor bottomColorSensor;
    int colorARGB = 0;
    public I2cDeviceSynchImpl rangeSensorRight;
    public I2cDeviceSynchImpl rangeSensorLeft;
    I2cDevice rangeA;
    OpticalDistanceSensor odsLight;
    double lightFromThread = 0;
    boolean getLight = false;

    static final double WHITE_THRESHOLD = 0.3;
    int COLOR_SENSOR_THRESHOLD = 60000000;
    double DIST = 14;
    int whiteLineCount = 0;

    public MecanumAuto(FtcOpMode anOpMode) {
        opMode = anOpMode;
    }

    public double startShootingtime = 0;
    public double prevTime = 0;

    public void init(HardwareMap hardwareMap, Telemetry telem, AllianceColor allianceColor) throws InterruptedException {

        color = allianceColor;
        telemetry = telem;

        frontLeftMotor = new FtcDcMotor("motor_3");
        frontRightMotor = new FtcDcMotor("motor_2");
        backLeftMotor = new FtcDcMotor("motor_1");
        backRightMotor = new FtcDcMotor("motor_4");

        frontLeftMotor.setInverted(false);
        frontRightMotor.setInverted(true);
        backLeftMotor.setInverted(false);
        backRightMotor.setInverted(true);

        frontLeftMotor.resetPosition();
        frontRightMotor.resetPosition();
        backLeftMotor.resetPosition();
        backRightMotor.resetPosition();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        driveBase = new TrcDriveBaseIMU(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, imu);

        leftColorSensor = hardwareMap.colorSensor.get("lcs");
        I2cAddr i2cAddr = I2cAddr.create8bit(0x4c);
        leftColorSensor.setI2cAddress(i2cAddr);

        rightColorSensor = hardwareMap.colorSensor.get("rcs");

        bottomColorSensor = hardwareMap.colorSensor.get("bcs");
        i2cAddr = I2cAddr.create8bit(0x3e);
        bottomColorSensor.setI2cAddress(i2cAddr);
        bottomColorSensor.enableLed(true);

        rangeA = hardwareMap.i2cDevice.get("rightMR");// Primary LEGO Light Sensor
        rangeSensorRight = new I2cDeviceSynchImpl(rangeA, I2cAddr.create8bit(0x30), false);
        rangeSensorRight.engage();

        rangeA = hardwareMap.i2cDevice.get("leftMR");// Primary LEGO Light Sensor
        rangeSensorLeft = new I2cDeviceSynchImpl(rangeA, I2cAddr.create8bit(0x28), false);
        rangeSensorLeft.engage();

        odsLight = hardwareMap.opticalDistanceSensor.get("odsLight");

    }

    public double IMUheading() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    int getOpticalDistance(I2cDeviceSynchImpl rangeSensor) {
        return rangeSensor.read(0x04, 2)[1] & 0xFF;
    }

    public int getcmUltrasonic(I2cDeviceSynchImpl rangeSensor) {
        return rangeSensor.read(0x04, 2)[0] & 0xFF;
    }

    public void toWall() {
        while (opMode.opModeIsActive() && getcmUltrasonic(rangeSensorLeft) > DIST) {
            if (color == AllianceColor.BLUE)
                driveBase.mecanumDrive_Polar(.7, -120, 0);
            else if (color == AllianceColor.RED)
                driveBase.mecanumDrive_Polar(.7, -60, 0);
            telemetry.addData("Distance", getcmUltrasonic(rangeSensorLeft));
            telemetry.update();
            if (odsLight.getLightDetected() > WHITE_THRESHOLD) {
                telemetry.log().add("White detected");
                break;
            }
            DbgLog.msg("ODS Sensor Value " + odsLight.getLightDetected());
        }
        driveBase.stop();
    }

    Runnable lightFromOds = new Runnable() {
        @Override
        public void run() {
            while (opMode.opModeIsActive() && getLight) {
                lightFromThread = odsLight.getLightDetected();
            }
        }
    };
    Thread getOdsLight = new Thread(lightFromOds);

    Runnable colorSensorARGB = new Runnable() {
        @Override
        public void run() {
            while (opMode.opModeIsActive() && getLight) {
                //telemetry.addLine("Thread running");
                //telemetry.update();
                colorARGB = bottomColorSensor.argb();
            }
        }
    };
    Thread getARGB = new Thread(colorSensorARGB);

    public void toWhiteLine() throws InterruptedException {

        getLight = true;
        //getOdsLight.start();
        //getARGB.start();

        whiteLineCount++;

        if (whiteLineCount == 2) {
            if (color == AllianceColor.BLUE)
                driveBase.mecanumDrive_Polar(0.3, 180, 0);
            else if (color == AllianceColor.RED)
                driveBase.mecanumDrive_Polar(0.3, 0, 0);
            opMode.sleep(1000);
        }

        while (opMode.opModeIsActive() && odsLight.getLightDetected() < WHITE_THRESHOLD
                && bottomColorSensor.argb() < COLOR_SENSOR_THRESHOLD) {

            DbgLog.msg("ODS Sensor Value " + odsLight.getLightDetected());

            if (color == AllianceColor.BLUE) {
                if (getcmUltrasonic(rangeSensorLeft) < 8) { // too close
                    driveBase.mecanumDrive_Polar(.3, 90, 0); // move right
                } else if (getcmUltrasonic(rangeSensorLeft) > 44) {
                    driveBase.mecanumDrive_Polar(.3, -90, 0); // move left
                } else {
                    driveBase.mecanumDrive_Polar(.15, 180, 0); // drive forward
                }
            } else if (color == AllianceColor.RED) {
                if (getcmUltrasonic(rangeSensorLeft) < 8) { // too close
                    driveBase.mecanumDrive_Polar(.3, -90, 0); // move right
                } else if (getcmUltrasonic(rangeSensorLeft) > 44) {
                    driveBase.mecanumDrive_Polar(.3, 90, 0); // move left
                } else {
                    driveBase.mecanumDrive_Polar(.15, 0, 0); // drive forward
                }
            }

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level", odsLight.getLightDetected());
            telemetry.addData("Color", bottomColorSensor.argb());
            telemetry.addData("Distance", getcmUltrasonic(rangeSensorLeft));
            telemetry.update();
            opMode.idle();

        }
        telemetry.log().add("Line 1");

        driveBase.stop();
        opMode.sleep(200);
        if (color == AllianceColor.BLUE)
            driveBase.mecanumDrive_Polar(.08, 0, 0);
        else if (color == AllianceColor.RED)
            driveBase.mecanumDrive_Polar(.08, 180, 0, false);
        while (opMode.opModeIsActive() && odsLight.getLightDetected() < WHITE_THRESHOLD
                && bottomColorSensor.argb() < COLOR_SENSOR_THRESHOLD) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level", odsLight.getLightDetected());
            telemetry.addData("Color", bottomColorSensor.argb());
            telemetry.addData("Distance", getcmUltrasonic(rangeSensorLeft));
            telemetry.update();
            opMode.idle();

        }
        telemetry.log().add("Line 2");
        driveBase.stop();
        getLight = false;
    }

    public void pushButton() throws InterruptedException {

        telemetry.log().add("in the push button method");

        telemetry.update();
        leftColorSensor.enableLed(true);
        rightColorSensor.enableLed(true);

        telemetry.update();

        ElapsedTime time = new ElapsedTime();
        time.reset();

        boolean wrongColor;
        if (getcmUltrasonic(rangeSensorRight) > 9) {
            telemetry.log().add("too far");
            while (opMode.opModeIsActive() && getcmUltrasonic(rangeSensorRight) > 9) {
                driveBase.mecanumDrive_Polar(.6, 90, 0);
            }
        }

        int frontLeftPos = frontLeftMotor.motor.getCurrentPosition();
        int frontRightPos = frontRightMotor.motor.getCurrentPosition();
        int backLeftPos = backLeftMotor.motor.getCurrentPosition();
        int backRightPos = backRightMotor.motor.getCurrentPosition();

        do {
            telemetry.addData("Time", time.seconds());
            telemetry.log().add("in the push button method while loop");
            telemetry.addLine("Left blue: " + leftColorSensor.blue() + " | Left red: " + leftColorSensor.red());
            telemetry.addLine("Right blue: " + rightColorSensor.blue() + " | Right red: " + rightColorSensor.red());

            telemetry.update();

            wrongColor = false;

            if (color == AllianceColor.BLUE) {

                if (leftColorSensor.blue() > rightColorSensor.blue()) { // && !verifyBlue()){
                    //write the code here to press the left button
                    telemetry.log().add("left is blue");
                    telemetry.update();
                    driveBase.mecanumDrive_Polar(.4, 90, -.1);

                } else if (rightColorSensor.blue() > leftColorSensor.blue()) {// && !verifyBlue()){
                    //write the code here to press the right button
                    telemetry.log().add("right is blue");
                    telemetry.update();
                    driveBase.mecanumDrive_Polar(.4, 90, .1);

                } else if (leftColorSensor.red() > leftColorSensor.blue() &&
                        rightColorSensor.red() > rightColorSensor.blue()) {
                    //red button has been pressed
                    telemetry.log().add("beacon is red");
                    telemetry.update();
                    driveBase.mecanumDrive_Polar(.4, 90, 0);
                    // frontLeftMotor.setPower(-0.1);
                    // backLeftMotor.setPower(-0.1);
                    // frontRightMotor.setPower(-0.1);
                    // backRightMotor.setPower(-0.1);

                    //opMode.sleep(4000); // wait 5 seconds total

                    wrongColor = true;
                } else if (getcmUltrasonic(rangeSensorRight) > 10) {
                    telemetry.log().add("too far");
                    driveBase.mecanumDrive_Polar(.4, 90, 0);
                    // frontLeftMotor.setPower(-0.1);
                    // backLeftMotor.setPower(-0.1);
                    // frontRightMotor.setPower(-0.1);
                    // backRightMotor.setPower(-0.1);
                } else {
                    telemetry.log().add("blue is not detected");
                    telemetry.update();
                    break;
                }
            }
            else if (color == AllianceColor.RED) {
                if (leftColorSensor.red() > rightColorSensor.red()) {// && !verifyBlue()){
                    //write the code here to press the left button
                    telemetry.log().add("left is red");
                    telemetry.update();
                    // backRightMotor.setPower(-0.2);
                    // backLeftMotor.setPower(0.2);
                    driveBase.mecanumDrive_Polar(.4, 90, -.1);

                } else if (rightColorSensor.red() > leftColorSensor.red()) {// && !verifyBlue()){
                    //write the code here to press the right button
                    telemetry.log().add("right is red");
                    telemetry.update();
                    // frontLeftMotor.setPower(-0.2);
                    // frontRightMotor.setPower(0.2);
                    driveBase.mecanumDrive_Polar(.4, 90, .1);

                } else if (leftColorSensor.blue() > leftColorSensor.red()
                        && rightColorSensor.blue() > rightColorSensor.red()) {
                    //blue button has been pressed
                    telemetry.log().add("beacon is blue");
                    telemetry.update();
                    driveBase.mecanumDrive_Polar(.4, 90, 0);
                    // frontLeftMotor.setPower(-0.1);
                    // backLeftMotor.setPower(-0.1);
                    // frontRightMotor.setPower(-0.1);
                    // backRightMotor.setPower(-0.1);

                    //opMode.sleep(4000); // wait 5 seconds total

                    wrongColor = true;
                } else if (getcmUltrasonic(rangeSensorRight) > 10) {
                    telemetry.log().add("too far");
                    driveBase.mecanumDrive_Polar(.4, 90, 0);
                    // frontLeftMotor.setPower(-0.1);
                    // backLeftMotor.setPower(-0.1);
                    // frontRightMotor.setPower(-0.1);
                    // backRightMotor.setPower(-0.1);

                } else {
                    telemetry.log().add("red is not detected");
                    telemetry.update();
                    break;
                }
            }
            telemetry.update();
            opMode.sleep(1000);

            // driveBase.mecanumDrive_Polar(0.2, 0, 0);
            // opMode.sleep(100);
            driveBase.stop();

            telemetry.addLine("Left blue: " + leftColorSensor.blue() + " | Left red: " + leftColorSensor.red());
            telemetry.addLine("Right blue: " + rightColorSensor.blue() + " | Right red: " + rightColorSensor.red());
            telemetry.update();

            opMode.idle();

        } while (opMode.opModeIsActive() && !verify()
                && (time.seconds() < 4 || wrongColor));

        runToPos(frontLeftPos, frontRightPos, backLeftPos, backRightPos, 0.3);
        telemetry.log().add("end of the push button method");
    }

    public void runToPos(int frontLeftPos, int frontRightPos, int backLeftPos, int backRightPos, double power) {

        frontLeftMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.motor.setTargetPosition(frontLeftPos);
        frontRightMotor.motor.setTargetPosition(frontRightPos);
        backLeftMotor.motor.setTargetPosition(backLeftPos);
        backRightMotor.motor.setTargetPosition(backRightPos);

        frontLeftMotor.motor.setPower(power);
        frontRightMotor.motor.setPower(power);
        backLeftMotor.motor.setPower(power);
        backRightMotor.motor.setPower(power);

        while (opMode.opModeIsActive()
                && frontLeftMotor.motor.isBusy()
                && frontRightMotor.motor.isBusy()
                && backLeftMotor.motor.isBusy()
                && backRightMotor.motor.isBusy()) {
            telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d",
                    frontLeftPos, frontRightPos, backLeftPos, backRightPos);
            telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                    frontLeftMotor.motor.getCurrentPosition(),
                    frontRightMotor.motor.getCurrentPosition(),
                    backLeftMotor.motor.getCurrentPosition(),
                    backRightMotor.motor.getCurrentPosition()
            );
        }

        frontLeftMotor.motor.setPower(0);
        frontRightMotor.motor.setPower(0);
        backLeftMotor.motor.setPower(0);
        backRightMotor.motor.setPower(0);

        frontLeftMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void turn(int turnAngle) throws InterruptedException {

        // leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // int leftPos = leftMotor.getCurrentPosition();
        // int rightPos = rightMotor.getCurrentPosition();

        double startAngle = IMUheading();
        angleZ = IMUheading();

        double angDiff = (turnAngle - angleZ) % 360;
        if (360 - Math.abs(angDiff) < Math.abs(angDiff))
            angDiff = -(360 * Math.signum(angDiff) - angDiff);

        telemetry.log().add("Angle Difference: " + angDiff);
        telemetry.update();

        if (angDiff < 0) { //turns right
            //leftMotor.setPower(APPROACH_SPEED * .6 );
            //rightMotor.setPower(-APPROACH_SPEED * .6);

            while (opMode.opModeIsActive() && angDiff < 0) {

                angleZ = IMUheading();
                angDiff = (turnAngle - angleZ) % 360;
                if (360 - Math.abs(angDiff) < Math.abs(angDiff))
                    angDiff = -(360 * Math.signum(angDiff) - angDiff);

                telemetry.addData("Angle", angleZ);
                telemetry.addData("Difference", angDiff);
                telemetry.update();

                frontLeftMotor.setPower(turnPower(angDiff));
                frontRightMotor.setPower(-turnPower(angDiff));
                backLeftMotor.setPower(turnPower(angDiff));
                backRightMotor.setPower(-turnPower(angDiff));
                telemetry.addData("Power", frontLeftMotor.getPower());

                // driveBase.mecanumDrive_Polar(turnPower(angDiff), 0, -90, false);
                // driveBase.mecanumDrive_Polar(turnPower(angDiff), 0, angDiff);

                /* if (leftMotor.getCurrentPosition() - 100 > leftPos
                        && rightMotor.getCurrentPosition() + 100 < rightPos
                        && IMUheading() == startAngle) {
                    resetIMuandPos(leftPos, rightPos);
                } */

                opMode.idle(); // Always call opMode.idle() at the bottom of your while(opModeIsActive()) loop
            }
        } else if (angDiff > 0) { //turns left
            //leftMotor.setPower(-APPROACH_SPEED);
            //rightMotor.setPower(APPROACH_SPEED);

            while (opMode.opModeIsActive() && angDiff > 0) {

                angleZ = IMUheading();
                angDiff = (turnAngle - angleZ) % 360;
                if (360 - Math.abs(angDiff) < Math.abs(angDiff))
                    angDiff = -(360 * Math.signum(angDiff) - angDiff);

                telemetry.addData("Angle", angleZ);
                telemetry.addData("Difference", angDiff);
                telemetry.update();


                frontLeftMotor.setPower(turnPower(angDiff));
                frontRightMotor.setPower(-turnPower(angDiff));
                backLeftMotor.setPower(turnPower(angDiff));
                backRightMotor.setPower(-turnPower(angDiff));
                telemetry.addData("Power", frontLeftMotor.getPower());

                // driveBase.mecanumDrive_Polar(turnPower(angDiff), 0, 90, false);
                // driveBase.mecanumDrive_Polar(turnPower(angDiff), 0, angDiff);

                /* if (leftMotor.getCurrentPosition() + 100 < leftPos
                        && rightMotor.getCurrentPosition() - 100 > rightPos
                        && IMUheading() == startAngle) {
                    resetIMuandPos(leftPos, rightPos);
                } */

                opMode.idle(); // Always call opMode.idle() at the bottom of your while(opModeIsActive()) loop
            }
        }

        driveBase.stop();
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    double turnPower(double difference) {
        if (Math.abs(difference) < 20) {
            return 0.1;
        } else if (Math.abs(difference) < 45) {
            return 0.15;
        } else if (Math.abs(difference) < 90) {
            return 0.3;
        } else return 0.4;
    }

    void resetIMuandPos(int left, int right) throws InterruptedException { //resets IMU to 0 at starting position of turn

        telemetry.addLine("IMU Problem");
        telemetry.update();

        // leftMotor.setPower(0);
        // rightMotor.setPower(0);

        // leftMotor.setTargetPosition(left);
        // rightMotor.setTargetPosition(right);

        // leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // rightMotor.setPower(.2);
        // leftMotor.setPower(.2);

        // while (opMode.opModeIsActive() &&
                // (leftMotor.isBusy() && rightMotor.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d", left, right);
        // telemetry.addData("Path2", "Running at %7d :%7d",
                    //          leftMotor.getCurrentPosition(),
                    // rightMotor.getCurrentPosition());
            telemetry.update();

        // opMode.idle();
        // }

        // rightMotor.setPower(0);
        // leftMotor.setPower(0);

        // leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Initializing IMU");
        telemetry.update();
        imu.initialize();
    }

    public boolean verify() {
        if (leftColorSensor.alpha() == 255 || rightColorSensor.alpha() == 255)
            throw new RuntimeException("Color Sensor problems");
        if (color == AllianceColor.BLUE) {
            if (leftColorSensor.blue() > leftColorSensor.red() && rightColorSensor.blue() > rightColorSensor.red()) {
                telemetry.addLine("Beacon is blue");
                return true;
            }
            telemetry.addLine("Beacon is red");
        }
        else if (color == AllianceColor.RED) {
            if (leftColorSensor.red() > leftColorSensor.blue() && rightColorSensor.red() > rightColorSensor.blue()) {
                telemetry.addLine("Beacon is red");
                return true;
            }
            telemetry.addLine("Beacon is blue");
        }
        telemetry.update();
        return false;
    }

    public void backup() {
        if (color == AllianceColor.BLUE)
            driveBase.mecanumDrive_Polar(0.2, 90, 0, false);
        else if (color == AllianceColor.RED)
            driveBase.mecanumDrive_Polar(0.2, -90, 0, false);
        while (opMode.opModeIsActive() && getcmUltrasonic(rangeSensorRight) < DIST) {
            telemetry.addData("Distance", getcmUltrasonic(rangeSensorRight));
        }
        driveBase.stop();
    }
}