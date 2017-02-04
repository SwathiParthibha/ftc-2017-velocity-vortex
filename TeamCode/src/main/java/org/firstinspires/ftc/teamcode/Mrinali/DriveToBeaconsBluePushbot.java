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

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="Beacons Autonomous Blue", group="Pushbot")
@Disabled
public class DriveToBeaconsBluePushbot extends LinearOpMode {

    //To change red to blue: negative angles, color sensors sense blue, right side range sensor

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
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
    double backup = -2; //distance in in to back up after pressing beacon
    double overBeacon1 = 2.5;
    double overBeacon2 = 2;
    byte[] rangeSensorCache;
    byte[] sideRangeSensorCache;
    I2cDevice rangeA;
    I2cDevice rangeB;

    @Override
    public void runOpMode() throws InterruptedException {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to our Light Sensor object.
        lightSensor = hardwareMap.lightSensor.get("light sensor");
        rangeA = hardwareMap.i2cDevice.get("range sensor");// Primary LEGO Light Sensor
        rangeSensor = new I2cDeviceSynchImpl(rangeA, I2cAddr.create8bit(0x2a), false);
        rangeA = hardwareMap.i2cDevice.get("r side range");// Primary LEGO Light Sensor
        sideRangeSensor = new I2cDeviceSynchImpl(rangeA, I2cAddr.create8bit(0x28), false);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        rangeSensor.engage();
        sideRangeSensor.engage();

        //angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        //origAngle = angles.firstAngle;

        leftColorSensor  = hardwareMap.colorSensor.get("lcs");
        I2cAddr i2cAddr = I2cAddr.create8bit(0x4c);
        leftColorSensor.setI2cAddress(i2cAddr);

        rightColorSensor = hardwareMap.colorSensor.get("rcs");

        verify();
        telemetry.addData("verifyBlue", verify()); //checks color sensors

        // turn on LED of light sensor.
        lightSensor.enableLed(true);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to runIMU");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {

            // Display the light level while we are waiting to start
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.addData("Front Ultrasonic", getcmUltrasonic(rangeSensor));
            angleZ = IMUheading();
            telemetry.addData("Side Ultrasonic", getcmUltrasonic(sideRangeSensor));
            telemetry.addData("Angle", angleZ);
            //telemetry.addData("verifyBlue", verifyBlue());
            telemetry.addData("leftColorSensor", leftColorSensor.argb());
            telemetry.addData("rightColorSensor", rightColorSensor.argb());
            telemetry.update();
            idle();
        }

        encoderDrive(APPROACH_SPEED, 3, 3, 3);
        turn(-40); //The robot uses the IMU to turn to 40 degrees
        encoderDrive(APPROACH_SPEED, 17, 17, 7);
        toWhiteLine(false); //and then proceeds to the white line using encoders and a NXT light sensor
        turn(-90); //The robot then turns perpendicular to the beacon suing the IMU
        sleep(100);
        approachBeacon(); //and advances until it is 8 cm from the beacon which is measured using a range sensor
        pushButton(); //The robot then uses two color sensors to push the blue side of the beacon, and verifies it press the correct side. If it didn't, then it will wait for 5 seconds and try again.
        encoderDrive(APPROACH_SPEED, backup, backup, 3); //The robot then moves backward using encoders
        turn(0); //and turns parallel to the beacon using the IMU
        encoderDrive(APPROACH_SPEED, 8, 8, 5);
        //maintainDist(); //maintains a certain distance from the wall using a range sensor and the IMU

        turn(0);
        robot.leftMotor.setPower(APPROACH_SPEED * .4);
        robot.rightMotor.setPower(APPROACH_SPEED * .4);
        toWhiteLine(true); //It advances to the next white line
        sleep(100);
        turn(-90); //It turn perpendicular to the beacon again using the IMU sensor
        approachBeacon(); //then approaches the beacon and stops 8 cm from beacon again
        pushButton(); //It uses two color sensors to push the blue side of the beacon, and verifies it press the correct side. If it didn't, then it will wait for 5 seconds and try again
        encoderDrive(APPROACH_SPEED, backup, backup, 3); //Then it will back up

        robot.leftMotor.setPower(APPROACH_SPEED); //and turns until it is facing the cap ball
        robot.rightMotor.setPower(-APPROACH_SPEED);
        while (angleZ > -180 && angleZ < 0 || angleZ > 155) {
            angleZ = IMUheading();
            telemetry.addData("Angle", angleZ);
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        encoderDrive(APPROACH_SPEED, 24, 24, 5); //The robot then advances forward, using encoders, and hits the cap ball off the centerpiece and parks on it
    }

    double IMUheading() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    private int getOpticalDistance(I2cDeviceSynchImpl rangeSensor) {
        return rangeSensor.read(0x04, 2)[1]  & 0xFF;
    }

    private int getcmUltrasonic(I2cDeviceSynchImpl rangeSensor){
        return rangeSensor.read(0x04, 2)[0]  & 0xFF;
    }

    void toWhiteLine(boolean wall) throws InterruptedException {
        // Start the robot moving forward, and then begin looking for a white line.
        if (!wall) {
            robot.leftMotor.setPower(APPROACH_SPEED * .4);
            robot.rightMotor.setPower(APPROACH_SPEED * .4);
        }

        while (opModeIsActive() && (lightSensor.getLightDetected() < WHITE_THRESHOLD)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();
            idle();
        }

        // Stop all motors
        stopRobot();

        if (!wall) {
            encoderDrive(APPROACH_SPEED * .4, overBeacon1, overBeacon1, 2);
        }
        else {
            encoderDrive(APPROACH_SPEED * .4, overBeacon2, overBeacon2, 2);
        }
    }

    /*void turn(int turnAngle)
    {
        angleZ = IMUheading();

        if (turnAngle < angleZ) {
            robot.leftMotor.setPower(APPROACH_SPEED);
            robot.rightMotor.setPower(-APPROACH_SPEED);

            while (opModeIsActive() && (turnAngle + 7 < angleZ)) {

                angleZ = IMUheading();
                telemetry.addData("Angle", angleZ);
                telemetry.update();
                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            }
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
        }

        else if (turnAngle > angleZ) {
            robot.leftMotor.setPower(-APPROACH_SPEED);
            robot.rightMotor.setPower(APPROACH_SPEED);

            while (opModeIsActive() && (turnAngle  - 7 > angleZ)) {

                angleZ = IMUheading();
                telemetry.addData("Angle", angleZ);
                telemetry.update();
                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            }
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
        }
    }*/
    void turn(int turnAngle)
    {
        //robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        angleZ = IMUheading();

        double angDiff = turnAngle-angleZ; //positive: turn left
        //if (Math.abs(angDiff) > 180) angDiff = angDiff % 180;

        if (angDiff < 0) { //turns right
            robot.leftMotor.setPower(APPROACH_SPEED * .6 );
            robot.rightMotor.setPower(-APPROACH_SPEED * .6);

            while (opModeIsActive() && (angDiff < 0)) {

                angleZ = IMUheading();
                angDiff = turnAngle-angleZ;

                if (Math.abs(angDiff) < 90) {
                    robot.leftMotor.setPower(APPROACH_SPEED * .2);
                    robot.rightMotor.setPower(-APPROACH_SPEED * .2);
                }
                else if (Math.abs(angDiff) < 45) {
                    robot.leftMotor.setPower(APPROACH_SPEED * .05);
                    robot.rightMotor.setPower(-APPROACH_SPEED * .05);
                }

                telemetry.addData("Angle", angleZ);
                telemetry.update();
                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            }
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
        }

        else if (angDiff > 0) { //turns left
            robot.leftMotor.setPower(-APPROACH_SPEED);
            robot.rightMotor.setPower(APPROACH_SPEED);

            while (opModeIsActive() && (angDiff > 0)) {

                angleZ = IMUheading();
                angDiff = turnAngle-angleZ;

                if (Math.abs(angDiff) < 90) {
                    robot.leftMotor.setPower(-APPROACH_SPEED * .2);
                    robot.rightMotor.setPower(APPROACH_SPEED * .2);
                }
                else if (Math.abs(angDiff) < 45) {
                    robot.leftMotor.setPower(-APPROACH_SPEED * .05);
                    robot.rightMotor.setPower(APPROACH_SPEED * .05);
                }

                telemetry.addData("Angle", angleZ);
                telemetry.update();
                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            }
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
        }
        //robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void approachBeacon()
    {
        robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Momentarily stop
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        sleep(200);

        telemetry.addData("Distance", getcmUltrasonic(rangeSensor));
        telemetry.update();

        if (getcmUltrasonic(rangeSensor) > DIST * 2) {
            robot.leftMotor.setPower(APPROACH_SPEED * .8);
            robot.rightMotor.setPower(APPROACH_SPEED * .8);

            while (opModeIsActive() && getcmUltrasonic(rangeSensor) > DIST * 2) {

                telemetry.log().add("Left power" + robot.leftMotor.getPower());
                telemetry.log().add("Right power" + robot.rightMotor.getPower());
                telemetry.addData("Distance", getcmUltrasonic(rangeSensor));
                telemetry.update();

                idle();
            }
            //Momentarily stop
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            sleep(100);
        }

        if (getcmUltrasonic(rangeSensor) > DIST) {
            robot.leftMotor.setPower(APPROACH_SPEED * .4);
            robot.rightMotor.setPower(APPROACH_SPEED * .4);
            while (opModeIsActive() && getcmUltrasonic(rangeSensor) > DIST) {

                telemetry.log().add("Left power" + robot.leftMotor.getPower());
                telemetry.log().add("Right power" + robot.rightMotor.getPower());
                telemetry.addData("Distance", getcmUltrasonic(rangeSensor));
                telemetry.update();

                idle();
            }
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            sleep(100);
        }

        telemetry.addData("Distance", getcmUltrasonic(rangeSensor));
        telemetry.update();
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    void pushButton() {

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

                robot.rightMotor.setPower(APPROACH_SPEED); //motors seem to work in reverse
                robot.leftMotor.setPower(0);
            } else if(rightColorSensor.blue() > leftColorSensor.blue()) {// && !verifyBlue()){
                //write the code here to press the right button
                telemetry.log().add("right is blue");
                telemetry.update();

                robot.leftMotor.setPower(APPROACH_SPEED); //motors seem to work in reverse
                robot.rightMotor.setPower(0);
            } else if(leftColorSensor.red() > leftColorSensor.blue() &&
                    rightColorSensor.red() > rightColorSensor.blue()) {
                //red button has been pressed
                telemetry.log().add("beacon is red");
                telemetry.update();

                //sleep(4000); // wait 5 seconds total
                robot.leftMotor.setPower(APPROACH_SPEED);
                robot.rightMotor.setPower(0);
            } else if(getcmUltrasonic(rangeSensor) > 8) {
                encoderDrive(APPROACH_SPEED, 1, 1, 1);
            } else{
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
                telemetry.log().add("blue is not detected");
                telemetry.update();
                break;
            }
            telemetry.update();
            sleep(1500);
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            //robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            robot.rightMotor.setPower(-APPROACH_SPEED * .8);
            robot.leftMotor.setPower(-APPROACH_SPEED * .8);
            sleep(80);
            robot.rightMotor.setPower(0);
            robot.leftMotor.setPower(0);

            telemetry.addData("Left blue: ", leftColorSensor.blue());
            telemetry.addData("Right blue: ", rightColorSensor.blue());
            telemetry.update();

            //robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            //robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } while  (!verify() && opModeIsActive());

        telemetry.log().add("end of the push button method");

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    private boolean verify() {
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

    void maintainDist() {

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        sideRange = getcmUltrasonic(sideRangeSensor);
        angleZ = IMUheading();
        telemetry.addData("Side Range: ", getcmUltrasonic(sideRangeSensor));
        telemetry.addData("Angle", angleZ);
        telemetry.update();
        double distCorrect = SIDE_DIST - sideRange; //positive if too close
        if (SIDE_DIST - 5 > sideRange) //too close
            distCorrect = SIDE_DIST - sideRange + 5;
        else if (SIDE_DIST + 5 < sideRange) //too far
            distCorrect = SIDE_DIST - sideRange - 5;

        //makes angle closer to 0
        robot.leftMotor.setPower(APPROACH_SPEED * .6 + angleZ/50 - distCorrect/60);
        robot.rightMotor.setPower(APPROACH_SPEED * .6 - angleZ/50 + distCorrect/60);
    }

    public void drive(double distance, double speed) throws InterruptedException
    {
        //1220 ticks per rotation
        //how many rotations? depends on distance
        //distance in cm - convert distance to encoder ticks
        //distance for each encoder tick == circumference / 1220
        //target distance / distance for each encoder tick == number of encoder ticks needed
        telemetry.addData("Starting to Drive", robot.leftMotor.getCurrentPosition() / ROTATION);
        telemetry.update();

        runUsingEncoder();

        stopAndResetEncoder();

        robot.leftMotor.setPower(speed);
        robot.rightMotor.setPower(speed);

        robot.leftMotor.setTargetPosition(CMtoEncoderTicks(distance));
        robot.rightMotor.setTargetPosition(CMtoEncoderTicks(distance));

        runToPosition();

        while (robot.leftMotor.isBusy() && robot.rightMotor.isBusy() && opModeIsActive())
        {
            //telemetry.addData("Heading", heading);
        }

        stopRobot();

        runUsingEncoder();

        telemetry.addData("Finished Driving", robot.leftMotor.getCurrentPosition() / ROTATION);
        telemetry.update();
    }

    int CMtoEncoderTicks(double cm) {
        return (int) (cm * ROTATION / WHEEL_SIZE_IN / Math.PI);
        //(target dist * ticks per rotation) / (circumference)
    }

    public void stopRobot()
    {
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    public void stopAndResetEncoder()
    {
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runToPosition()
    {
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runUsingEncoder()
    {
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {

        ElapsedTime runtime = new ElapsedTime();
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.addData("Left motor busy", robot.leftMotor.isBusy());
                telemetry.addData("Right motor busy", robot.rightMotor.isBusy());
                telemetry.update();
            }
            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}