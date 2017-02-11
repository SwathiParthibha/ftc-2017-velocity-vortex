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
package org.firstinspires.ftc.teamcode.Saransh;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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

public class AutonomousActions extends LinearOpMode {

    //To change red to blue: negative angles, color sensors sense blue, right side range sensor

    /* Declare OpMode members. */
    //HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    LinearOpMode opMode;
    public DcMotor leftMotor   = null;
    public DcMotor rightMotor  = null;
    public DcMotor shooter1;
    public DcMotor shooter2;
    private boolean state;
    public DcMotor scooper;
    LightSensor lightSensor;      // Primary LEGO Light sensor,
    I2cDeviceSynchImpl rangeSensor;
    I2cDeviceSynchImpl sideRangeSensor;
    double sideRange;
    //ModernRoboticsI2cGyro gyro;   // Hardware Device Object
    ColorSensor leftColorSensor;
    ColorSensor rightColorSensor;
    BNO055IMU imu;
    Orientation angles;

    // OpticalDistanceSensor   lightSensor;   // Alternative MR ODS sensor
    double angleZ = 0;

    double angleInit = 0;

    static final double WHITE_THRESHOLD = 0.3;  // spans between 0.1 - 0.5 from dark to light
    static final double APPROACH_SPEED = 0.5;
    double WHEEL_SIZE_IN = 4;
    public int ROTATION = 1220; // # of ticks for 40-1 gear ratio
    static final double     DRIVE_GEAR_REDUCTION    = 1.5 ;     // This is < 1.0 if geared UP
    double GEAR_RATIO = 40;
    double     COUNTS_PER_INCH         = (ROTATION * DRIVE_GEAR_REDUCTION) /
            (WHEEL_SIZE_IN * Math.PI) * (40 / GEAR_RATIO);
    double DIST = 18;
    double SIDE_DIST = 30;
    double backup = -2;
    double overBeacon1 = 2;
    double overBeacon2 = 2;
    byte[] rangeSensorCache;
    byte[] sideRangeSensorCache;
    I2cDevice rangeA;
    I2cDevice rangeB;

    private boolean USE_TELEMETRY=false;

    shooterSettings RPM955;
    shooterSettings RPM0;
    shooterSettings RPM800;

    public AutonomousActions(LinearOpMode anOpMode) {
        opMode = anOpMode;
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public double startShootingtime=0;
    public double prevTime=0;

    public void init(HardwareMap hardwareMap, Telemetry telem) {

        // Define and Initialize Motors
        leftMotor   = hardwareMap.dcMotor.get("l");
        rightMotor  = hardwareMap.dcMotor.get("r");

        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        //armMotor.setPower(0);

        // Set all motors to runIMU without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1 = hardwareMap.dcMotor.get("shooter1");
        shooter2 = hardwareMap.dcMotor.get("shooter2");
        scooper = hardwareMap.dcMotor.get("scooper");

        state = false;

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry = telem;
        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to our Light Sensor object.
        lightSensor = hardwareMap.lightSensor.get("light sensor");
        rangeA = hardwareMap.i2cDevice.get("range sensor");// Primary LEGO Light Sensor
        rangeSensor = new I2cDeviceSynchImpl(rangeA, I2cAddr.create8bit(0x2a), false);
        rangeA = hardwareMap.i2cDevice.get("r side range");// Primary LEGO Light Sensor
        sideRangeSensor = new I2cDeviceSynchImpl(rangeA, I2cAddr.create8bit(0x28), false);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        rangeSensor.engage();
        sideRangeSensor.engage();

        //angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        //origAngle = angles.firstAngle;

        leftColorSensor  = hardwareMap.colorSensor.get("lcs");
        I2cAddr i2cAddr = I2cAddr.create8bit(0x4c);
        leftColorSensor.setI2cAddress(i2cAddr);

        rightColorSensor = hardwareMap.colorSensor.get("rcs");

        lightSensor.enableLed(true);

        RPM955= new shooterSettings();//default settings are for 955, 0.43,0.43
        RPM0 = new shooterSettings(0,0,0);
        RPM800 = new shooterSettings(800,0.35,0.35);

    }

    double IMUheading() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));

    }

    int getOpticalDistance(I2cDeviceSynchImpl rangeSensor) {
        return rangeSensor.read(0x04, 2)[1]  & 0xFF;
    }

    int getcmUltrasonic(I2cDeviceSynchImpl rangeSensor){
        return rangeSensor.read(0x04, 2)[0]  & 0xFF;
    }

    void toWhiteLine(boolean wall) throws InterruptedException {
        // Start the robot moving forward, and then begin looking for a white line.
        if (!wall) {
            leftMotor.setPower(APPROACH_SPEED * .4);
            rightMotor.setPower(APPROACH_SPEED * .4);
        }

        while (opMode.opModeIsActive() && lightSensor.getLightDetected() < WHITE_THRESHOLD) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();
            idle();
        }

        // Stop all motors
        stopRobot();

        if (!wall) {
            encoderDrive(APPROACH_SPEED * .4, 1, 1, 1);
        }
        else
            encoderDrive(APPROACH_SPEED * .4, 2, 2, 2);
    }

    void turn (int turnAngle) throws InterruptedException{
        //leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        angleZ = IMUheading();

        double angDiff = turnAngle-angleZ; //positive: turn left
        //if (Math.abs(angDiff) > 180) angDiff = angDiff % 180;

        if (angDiff < 0) { //turns right
            leftMotor.setPower(APPROACH_SPEED * .6 );
            rightMotor.setPower(-APPROACH_SPEED * .6);

            while (opMode.opModeIsActive() && angDiff < 0) {

                angleZ = IMUheading();
                angDiff = turnAngle-angleZ;
                ElapsedTime timer = new ElapsedTime();
                timer.reset();

                if (Math.abs(angDiff) < 90) {
                    if ((angleInit != 0 && timer.milliseconds() < 1000) && (leftMotor.isBusy() && rightMotor.isBusy()))
                    {
                        leftMotor.setPower(APPROACH_SPEED * .2);
                        rightMotor.setPower(-APPROACH_SPEED * .2);
                        timer.reset();
                    }

                }
                else if (Math.abs(angDiff) < 45) {
                    if ((angleInit == 0 && timer.milliseconds() > 1500) && (leftMotor.isBusy() || rightMotor.isBusy()))
                    {
                        leftMotor.setPower(APPROACH_SPEED * .05);
                        rightMotor.setPower(-APPROACH_SPEED * .05);
                    }
                }

                telemetry.addData("Angle", angleZ);
                telemetry.update();
                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            }
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }

        else if (angDiff > 0) { //turns left
            leftMotor.setPower(-APPROACH_SPEED);
            rightMotor.setPower(APPROACH_SPEED);

            while (angDiff > 0) {

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
                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            }
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
        //leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void approachBeacon() {
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Momentarily stop
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sleep(200);

        telemetry.addData("Distance", getcmUltrasonic(rangeSensor));
        telemetry.update();

        if (getcmUltrasonic(rangeSensor) > DIST * 2) {
            leftMotor.setPower(APPROACH_SPEED * .8);
            rightMotor.setPower(APPROACH_SPEED * .8);

            while (getcmUltrasonic(rangeSensor) > DIST * 2) {

                telemetry.log().add("Left power" + leftMotor.getPower());
                telemetry.log().add("Right power" + rightMotor.getPower());
                telemetry.addData("Distance", getcmUltrasonic(rangeSensor));
                telemetry.update();

                idle();
            }
            //Momentarily stop
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            sleep(100);
        }

        if (getcmUltrasonic(rangeSensor) > DIST) {
            leftMotor.setPower(APPROACH_SPEED * .4);
            rightMotor.setPower(APPROACH_SPEED * .4);
            while (getcmUltrasonic(rangeSensor) > DIST) {

                telemetry.log().add("Left power" + leftMotor.getPower());
                telemetry.log().add("Right power" + rightMotor.getPower());
                telemetry.addData("Distance", getcmUltrasonic(rangeSensor));
                telemetry.update();

                idle();
            }
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            sleep(100);
        }

        telemetry.addData("Distance", getcmUltrasonic(rangeSensor));
        telemetry.update();
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    void followLineBlueSide() throws InterruptedException {
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addLine("Following Line");
        leftMotor.setPower(.2);
        rightMotor.setPower(-.2);
        while (opMode.opModeIsActive() && lightSensor.getLightDetected() < WHITE_THRESHOLD) {
            telemetry.addData("Light", lightSensor.getLightDetected());
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        while (opMode.opModeIsActive() && getcmUltrasonic(rangeSensor) > 11){
            telemetry.addData("Front range", getcmUltrasonic(rangeSensor));
            telemetry.addData("Light", lightSensor.getLightDetected());
            if(lightSensor.getLightDetected() > WHITE_THRESHOLD){
                telemetry.addLine("Moving right");
                leftMotor.setPower(0.2);
                rightMotor.setPower(0);
            } else {
                telemetry.addLine("Moving left");
                leftMotor.setPower(0);
                rightMotor.setPower(0.2);
            }
        }
        stopRobot();
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    void followLineRedSide() throws InterruptedException {
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addLine("Following Line");
        leftMotor.setPower(-.2);
        rightMotor.setPower(.2);
        while (opMode.opModeIsActive() && lightSensor.getLightDetected() < WHITE_THRESHOLD) {
            telemetry.addData("Light", lightSensor.getLightDetected());
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        while (opMode.opModeIsActive() && getcmUltrasonic(rangeSensor) > 11){
            telemetry.addData("Front range", getcmUltrasonic(rangeSensor));
            telemetry.addData("Light", lightSensor.getLightDetected());
            if(lightSensor.getLightDetected() > WHITE_THRESHOLD){
                telemetry.addLine("Moving left");
                leftMotor.setPower(0);
                rightMotor.setPower(0.2);
            } else {
                telemetry.addLine("Moving right");
                leftMotor.setPower(0.2);
                rightMotor.setPower(0);
            }
        }
        stopRobot();
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    void pushBlueButton() throws InterruptedException {

        telemetry.log().add("in the push button method");

        telemetry.update();
        leftColorSensor.enableLed(true);
        rightColorSensor.enableLed(true);

        telemetry.update();
        int leftBlue = leftColorSensor.blue();
        int rightBlue = rightColorSensor.blue();

        do{
            telemetry.log().add("in the push button method while loop");
            telemetry.addData("Left blue: ", leftColorSensor.blue());
            telemetry.addData("Right blue: ", rightColorSensor.blue());

            telemetry.update();

            if(leftColorSensor.blue() > rightColorSensor.blue()){// && !verifyBlue()){
                //write the code here to press the left button
                telemetry.log().add("left is blue");
                telemetry.update();

                rightMotor.setPower(APPROACH_SPEED); //motors seem to work in reverse
                leftMotor.setPower(0);
            } else if(rightColorSensor.blue() > leftColorSensor.blue()) {// && !verifyBlue()){
                //write the code here to press the right button
                telemetry.log().add("right is blue");
                telemetry.update();

                leftMotor.setPower(APPROACH_SPEED); //motors seem to work in reverse
                rightMotor.setPower(0);
            } else if(leftColorSensor.red() > leftColorSensor.blue() &&
                    rightColorSensor.red() > rightColorSensor.blue()){
                //red button has been pressed
                telemetry.log().add("beacon is red");
                telemetry.update();

                //sleep(4000); // wait 5 seconds total
                leftMotor.setPower(APPROACH_SPEED);
                rightMotor.setPower(0);

            } else if(getcmUltrasonic(rangeSensor) > 8) {
                encoderDrive(APPROACH_SPEED, 1, 1, 1);
            } else{
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                telemetry.log().add("blue is not detected");
                telemetry.update();
                break;
            }
            telemetry.update();
            sleep(1500);
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            //leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftMotor.setPower(-APPROACH_SPEED * .8);
            rightMotor.setPower(-APPROACH_SPEED * .8);
            sleep(40);
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            telemetry.addData("Left blue: ", leftColorSensor.blue());
            telemetry.addData("Right blue: ", rightColorSensor.blue());
            telemetry.update();

            //leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            //rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } while (opMode.opModeIsActive() && !verifyBlue());

        telemetry.log().add("end of the push button method");

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    void pushRedButton() throws InterruptedException {

        telemetry.log().add("in the push button method");

        telemetry.update();
        leftColorSensor.enableLed(true);
        rightColorSensor.enableLed(true);

        telemetry.update();
        int leftBlue = leftColorSensor.blue();
        int rightBlue = rightColorSensor.blue();

        do{
            telemetry.log().add("in the push button method while loop");
            telemetry.addData("Left red: ", leftColorSensor.red());
            telemetry.addData("Right red: ", rightColorSensor.red());

            telemetry.update();

            if(leftColorSensor.red() > rightColorSensor.red()){// && !verifyBlue()){
                //write the code here to press the left button
                telemetry.log().add("left is red");
                telemetry.update();

                rightMotor.setPower(APPROACH_SPEED); //motors seem to work in reverse
                leftMotor.setPower(0);
            } else if(rightColorSensor.red() > leftColorSensor.red()) {// && !verifyBlue()){
                //write the code here to press the right button
                telemetry.log().add("right is red");
                telemetry.update();

                leftMotor.setPower(APPROACH_SPEED); //motors seem to work in reverse
                rightMotor.setPower(0);
            } else if(leftColorSensor.blue() > leftColorSensor.red() && rightColorSensor.blue() > rightColorSensor.red()){
                //red button has been pressed
                telemetry.log().add("beacon is blue");
                telemetry.update();

                //sleep(4000); // wait 5 seconds total
                leftMotor.setPower(APPROACH_SPEED);
                rightMotor.setPower(0);
            } else if(getcmUltrasonic(rangeSensor) > 8) {
                encoderDrive(APPROACH_SPEED, 1, 1, 1);
            } else{
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                telemetry.log().add("red is not detected");
                telemetry.update();
                break;
            }
            telemetry.update();
            sleep(1500);
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            //leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightMotor.setPower(-APPROACH_SPEED * .8);
            leftMotor.setPower(-APPROACH_SPEED * .8);
            sleep(80);
            rightMotor.setPower(0);
            leftMotor.setPower(0);

            //leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            //rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            telemetry.addData("Left red: ", leftColorSensor.red());
            telemetry.addData("Right red: ", rightColorSensor.red());
            telemetry.update();
        } while  (opMode.opModeIsActive() && !verifyRed());

        telemetry.log().add("end of the push button method");

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    boolean verifyBlue() {
        if(leftColorSensor.alpha() == 255 || rightColorSensor.alpha() == 255)
            throw new RuntimeException("Color Sensor problems");
        /*else if (leftColorSensor.red() == rightColorSensor.red()
                && leftColorSensor.blue() == rightColorSensor.blue()
                && leftColorSensor.red() > 2
                && rightColorSensor.red() > 2)
            throw new RuntimeException("Color Sensor problems");*/

        if(leftColorSensor.blue() > leftColorSensor.red() && rightColorSensor.blue() > rightColorSensor.red()){
            telemetry.addLine("Beacon is blue");
            return true;
        }
        /*else if(Math.abs(leftColorSensor.blue() - rightColorSensor.blue()) < 2){
            return true;
        }*/
        telemetry.addLine("Beacon is red");
        return false;
    }

    boolean verifyRed() {
        if(leftColorSensor.alpha() == 255 || rightColorSensor.alpha() == 255)
            throw new RuntimeException("Color Sensor problems");
        /*else if (leftColorSensor.red() == rightColorSensor.red()
                && leftColorSensor.blue() == rightColorSensor.blue()
                && leftColorSensor.red() > 2
                && rightColorSensor.red() > 2)
            throw new RuntimeException("Color Sensor problems");*/

        if(leftColorSensor.red() > leftColorSensor.blue() && rightColorSensor.red() > rightColorSensor.blue()){
            telemetry.addLine("Beacon is red");
            return true;
        }
        /*else if(Math.abs(leftColorSensor.blue() - rightColorSensor.blue()) < 2){
            return true;
        }*/
        telemetry.addLine("Beacon is blue");
        return false;
    }

    void maintainDist() throws InterruptedException {

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sideRange = getcmUltrasonic(sideRangeSensor);
        angleZ = IMUheading();
        telemetry.addData("Side Range: ", getcmUltrasonic(sideRangeSensor));
        telemetry.addData("Angle", angleZ);
        telemetry.update();
        double distCorrect = SIDE_DIST - sideRange; //positive if too close

        //makes angle closer to 0
        leftMotor.setPower(APPROACH_SPEED * .6 + angleZ/50 - distCorrect/60);
        rightMotor.setPower(APPROACH_SPEED * .6 - angleZ/50 + distCorrect/60);

    }

    public void stopRobot() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void encoderDrive (double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        leftMotor.setTargetPosition(newLeftTarget);
        rightMotor.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();

        leftMotor.setPower(Math.abs(speed));
        rightMotor.setPower(Math.abs(speed));

        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (leftMotor.isBusy() && rightMotor.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    leftMotor.getCurrentPosition(),
                    rightMotor.getCurrentPosition());
            telemetry.addData("Left motor busy", leftMotor.isBusy());
            telemetry.addData("Right motor busy", rightMotor.isBusy());
            telemetry.update();
        }
        // Stop all motion;
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  sleep(250);   // optional pause after each move
    }

    public void shoot() {
        EncoderShooter(scaleShooterPower(0.9));//0.6//0.7
        sleep(2000);
        scooper.setPower(1);
        sleep(2500);
        EncoderShooter(0);
        scooper.setPower(0);
    }

    public void EncoderShooter(double speed) {
        shooter1.setPower(speed);
        shooter2.setPower(speed);
    }

    public double scaleShooterPower(double intialPower) {
        double MAX_VOLTAGE=13.7;
        double currentVoltage= hardwareMap.voltageSensor.get("drive").getVoltage();
        double scaledPower=MAX_VOLTAGE*intialPower/currentVoltage;
        telemetry.addData("Scaled power: ", scaledPower);
        return scaledPower;
    }

    public class shooterSettings{//data members can be replaced, but default values are for 1750 ETPS = 955 RPM

        public shooterSettings(){}
        public shooterSettings(double therequestedRPM, double theoriginalPWR1, double theoriginalPWR2){
            requestedRPM=therequestedRPM;
            requestedEncoderTicksPerSecond =requestedRPM*110/60;
            originalPWR1=theoriginalPWR1;
            originalPWR2=theoriginalPWR2;
            requiredPWR1=originalPWR1;
            requiredPWR2=originalPWR2;
        }

        private double requestedRPM =955;//955;
        private double requestedEncoderTicksPerSecond =requestedRPM*110/60;//1750

        //PID variables
        private double dt=0;
        private double previous_position1=0;
        private double current_position1=0;
        private double current_rpm1=0;
        private double previous_rpm1=0;
        private double error1=0;
        private double previous_error1=0;
        private double integral1=0;
        private double derivative1=0;
        private double adjustment1=0;
        private double previous_position2=0;
        private double current_position2=0;
        private double current_rpm2=0;
        private double previous_rpm2=0;
        private double error2=0;
        private double previous_error2=0;
        private double integral2=0;
        private double derivative2=0;
        private double adjustment2=0;

        //PID Constants
        double Kp = 0.000001;
        double Ki = 0.0000001;//0.00000001
        double Kd = 0.0000001;

        //Timing variables
        public double rampUpTime=1.5;

        //Power Variables
        public double originalPWR1=0.42;
        public double originalPWR2=0.42;
        public final double allowedPowerDifference=0.03;
        public double requiredPWR1=originalPWR1;
        public double requiredPWR2=originalPWR2;
        public double deadband=20;

        //Kalman Filter Variables
        double input1=0;
        double prevXk1=0;
        double prevPk1=1;
        double Xk1=0;
        double Pk1=1;
        double Kk1=0;
        double R1=0.2;

        double input2=0;
        double prevXk2=0;
        double prevPk2=1;
        double Xk2=0;
        double Pk2=1;
        double Kk2=0;
        double R2=0.2;
    }

    public void EncoderShooter(shooterSettings settings)
    {
        if(settings.requestedRPM!=0) {


            if(startShootingtime==-999) {//only update on first run
                startShootingtime = getRuntime();
            }

            settings.dt=getRuntime()-prevTime;
            if (settings.dt> 0.01) {//only update every 10ms
                settings.current_position1 = shooter1.getCurrentPosition();//MUST BE FIRST - time sensitive measurement
                settings.current_position2 = shooter2.getCurrentPosition();//MUST BE FIRST - time sensitive measurement
                prevTime = getRuntime();//MUST BE FIRST - time sensitive measurement

                updateRPM1and2(settings);

                if(getRuntime()-startShootingtime>settings.rampUpTime) {//only update Kalmin and PID after ramp up
                    timeUpdate(settings);
                    measurementUpdate(settings);


                    //DbgLog.msg("Time: "+getRuntime()+"RPM1: " + current_rpm1+"RPM2: " + current_rpm2);

                    PID1Update(settings);
                    PID2Update(settings);

                    applyAdjustment1(settings);
                    applyAdjustment2(settings);
                }
                clipPower1(settings);
                clipPower2(settings);

                previous1Update(settings);
                previous2Update(settings);
            }

            checkIfReadyToShoot(settings);
            if(USE_TELEMETRY) {
                outputTelemetry(settings);
            }
            shooter1.setPower(settings.requiredPWR1);
            shooter2.setPower(settings.requiredPWR2);
        }
        else
        {
            shooter1.setPower(0);
            shooter2.setPower(0);
            startShootingtime=-999;
            resetKalmin(settings);
            resetPID(settings);
        }
    }


    public void updateRPM1and2(shooterSettings settings){
        settings.current_rpm1 = (settings.current_position1 - settings.previous_position1) / (settings.dt);
        settings.current_rpm2 = (settings.current_position2 - settings.previous_position2) / (settings.dt);
    }

    public void PID1Update(shooterSettings settings){
        settings.error1=-(settings.Xk1- settings.requestedEncoderTicksPerSecond);
        settings.integral1 = settings.integral1 + settings.error1 * settings.dt;//calculate integral of error
        settings.derivative1 = (settings.error1 - settings.previous_error1) / settings.dt;//calculate derivative of data

        if(Math.abs(settings.error1)<settings.deadband)
        {
            settings.integral1=0;
            settings.derivative1=0;
        }

        settings.adjustment1 = settings.Kp * settings.error1 + settings.Kd*settings.derivative1 + settings.Ki*settings.integral1;// + Ki * integral1 + Kd * derivative1;//summation of PID
    }

    public void PID2Update(shooterSettings settings){

        settings.error2=-(settings.Xk2- settings.requestedEncoderTicksPerSecond);
        settings.integral2 = settings.integral2 + settings.error2 * settings.dt;//calculate integral of error
        settings.derivative2 = (settings.error2 - settings.previous_error2) / settings.dt;//calculate derivative of data

        if(Math.abs(settings.error2)<settings.deadband)
        {
            settings.integral2=0;
            settings.derivative2=0;
        }

        settings.adjustment2 = settings.Kp * settings.error2 + settings.Kd*settings.derivative2 + settings.Ki*settings.integral2;// + Ki * integral1 + Kd * derivative1;//summation of PID
    }

    public void previous1Update(shooterSettings settings){
        settings.previous_error1=settings.error1;
        settings.previous_position1 = settings.current_position1;
        settings.previous_rpm1 = settings.current_rpm1;
    }

    public void previous2Update(shooterSettings settings){
        settings.previous_error2=settings.error2;
        settings.previous_position2 = settings.current_position2;
        settings.previous_rpm2 = settings.current_rpm2;
    }

    public void applyAdjustment1(shooterSettings settings) {
        settings.requiredPWR1+=settings.adjustment1;
    }

    public void applyAdjustment2(shooterSettings settings) {
        settings.requiredPWR2+=settings.adjustment2;
    }

    public void clipPower1(shooterSettings settings){
        if(settings.requiredPWR1<settings.originalPWR1-settings.allowedPowerDifference)
        {
            settings.requiredPWR1=settings.originalPWR1-settings.allowedPowerDifference;
        }
        else if(settings.requiredPWR1>settings.originalPWR1+settings.allowedPowerDifference)
        {
            settings.requiredPWR1=settings.originalPWR1+settings.allowedPowerDifference;
        }
    }

    public void clipPower2(shooterSettings settings){
        if(settings.requiredPWR2<settings.originalPWR2-settings.allowedPowerDifference)
        {
            settings.requiredPWR2=settings.originalPWR2-settings.allowedPowerDifference;
        }
        else if(settings.requiredPWR2>settings.originalPWR2+settings.allowedPowerDifference)
        {
            settings.requiredPWR2=settings.originalPWR2+settings.allowedPowerDifference;
        }
    }

    public boolean checkIfReadyToShoot(shooterSettings settings) {
        if(Math.abs(settings.error1)<settings.deadband && Math.abs(settings.error2)<settings.deadband && getRuntime()-startShootingtime>settings.rampUpTime)
        {
            telemetry.addData("READY TO SHOOT", "");
            return true;
        }
        else
        {
            return false;
        }
    }

    public void outputTelemetry(shooterSettings settings) {
        telemetry.addData("requiredPWR1: ", String.format("%.4f", settings.requiredPWR1));
        telemetry.addData("requiredPWR2: ", String.format("%.4f", settings.requiredPWR2));
        telemetry.addData("adjustment1: ", settings.adjustment1);
        telemetry.addData("P1: ", settings.Kp*settings.error1);
        telemetry.addData("I1: ", settings.Ki*settings.integral1);
        telemetry.addData("D1: ", settings.Kd*settings.derivative1);
        telemetry.addData("adjustment2: ", settings.adjustment2);
        telemetry.addData("P2: ", settings.Kp*settings.error2);
        telemetry.addData("I2: ", settings.Ki*settings.integral2);
        telemetry.addData("D2: ", settings.Kd*settings.derivative2);
        telemetry.addData("curr1", settings.current_rpm1);
        telemetry.addData("curr2", settings.current_rpm2);
        telemetry.addData("Kalmin1", settings.Xk1);
        telemetry.addData("Kalmin2", settings.Xk2);
        telemetry.addData("K1", settings.Kk1);
        telemetry.addData("K2", settings.Kk2);
        telemetry.addData("Time: ", "" + getRuntime());
        telemetry.addData("ReqestedETPS", settings.requestedEncoderTicksPerSecond);

    }

    //Kalmin phase 1
    public void timeUpdate(shooterSettings settings){
        settings.input1=settings.current_rpm1;
        settings.prevXk1=settings.Xk1;
        settings.prevPk1=settings.Pk1;

        settings.input2=settings.current_rpm2;
        settings.prevXk2=settings.Xk2;
        settings.prevPk2=settings.Pk2;
    }

    //Kalmin phase 2
    public void measurementUpdate(shooterSettings settings){
        //RPM1 calculations
        settings.Kk1=settings.prevPk1/(settings.prevPk1+settings.R1);
        settings.Xk1=settings.prevXk1+settings.Kk1*(settings.input1-settings.prevXk1);
        settings.Pk1=(1-settings.Kk1)*settings.prevPk1;

        //RPM2 calculations
        settings.Kk2=settings.prevPk2/(settings.prevPk2+settings.R2);
        settings.Xk2=settings.prevXk2+settings.Kk2*(settings.input2-settings.prevXk2);
        settings.Pk2=(1-settings.Kk2)*settings.prevPk2;
    }

    public void resetKalmin(shooterSettings settings){
        settings.input1=0;
        settings.prevXk1=0;
        settings.prevPk1=1;
        settings.Xk1=0;
        settings.Pk1=1;
        // Kk1=0;

        settings.input2=0;
        settings.prevXk2=0;
        settings.prevPk2=1;
        settings.Xk2=0;
        settings.Pk2=1;
        // Kk2=0;
    }

    public void resetPID(shooterSettings settings){
        settings.previous_position1=0;
        settings.current_position1=0;
        settings.current_rpm1=0;
        settings.previous_rpm1=0;
        settings.error1=0;
        settings.previous_error1=0;
        settings.integral1=0;
        settings.derivative1=0;
        settings.adjustment1=0;
        settings.previous_position2=0;
        settings.current_position2=0;
        settings.current_rpm2=0;
        settings.previous_rpm2=0;
        settings.error2=0;
        settings.previous_error2=0;
        settings.integral2=0;
        settings.derivative2=0;
        settings.adjustment2=0;
    }
}