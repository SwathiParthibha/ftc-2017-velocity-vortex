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
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

@Autonomous(name="Old Blue", group="Pushbot")
@Disabled
public class OldBlue extends LinearOpMode {

    //To change red to blue: negative angles, color sensors sense blue, right side range sensor

    /* Declare OpMode members. */
    org.firstinspires.ftc.teamcode.Mrinali.HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    LightSensor lightSensor;      // Primary LEGO Light sensor,
    ModernRoboticsI2cRangeSensor rangeSensor;
    ModernRoboticsI2cRangeSensor sideRangeSensor;
    double sideRange;
    //ModernRoboticsI2cGyro gyro;   // Hardware Device Object
    ColorSensor leftColorSensor;
    ColorSensor rightColorSensor;
    BNO055IMU imu;
    Orientation angles;

    // OpticalDistanceSensor   lightSensor;   // Alternative MR ODS sensor
    double angleZ = 0;

    // get a reference to a Modern Robotics GyroSensor object.

    static final double WHITE_THRESHOLD = 0.3;  // spans between 0.1 - 0.5 from dark to light
    static final double APPROACH_SPEED = 0.5;
    double WHEEL_SIZE_IN = 4;
    public int ROTATION = 1220; // # of ticks
    double     COUNTS_PER_INCH         = ROTATION /
            (WHEEL_SIZE_IN * Math.PI);
    double DIST = 7;
    double SIDE_DIST = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        // start calibrating the gyro.
        /*telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        // make sure the gyro is calibrated.
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();
        */

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
        // robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to our Light Sensor object.
        lightSensor = hardwareMap.lightSensor.get("light sensor");                // Primary LEGO Light Sensor
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
        sideRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "r siderange");

        /*
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
        I2cAddr fRange = I2cAddr.create8bit(0x24);
        rangeSensor.setI2cAddress(fRange);

        sideRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "r side range");
        I2cAddr rRange = I2cAddr.create8bit(0x26);
        sideRangeSensor.setI2cAddress(rRange);
        */

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        //origAngle = angles.firstAngle;

        leftColorSensor  = hardwareMap.colorSensor.get("lcs");
        rightColorSensor = hardwareMap.colorSensor.get("rcs");
        I2cAddr i2cAddr = I2cAddr.create8bit(0x4c);
        rightColorSensor.setI2cAddress(i2cAddr);

        // turn on LED of light sensor.
        lightSensor.enableLed(true);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to runIMU");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {

            // Display the light level while we are waiting to start
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.addData("Distance", rangeSensor.getDistance(DistanceUnit.CM));
            angleZ = IMUheading();
            telemetry.addData("Angle", angleZ);
            telemetry.addData("Side Range Sensor", sideRangeSensor.getDistance(DistanceUnit.CM));

            //telemetry.addData("Angle", angleZ);
            //telemetry.addData("verifyBlue", verifyBlue());
            //telemetry.addData("leftColorSensor", leftColorSensor.argb());
            //telemetry.addData("rightColorSensor", rightColorSensor.argb());
            telemetry.update();
            idle();
        }



        toWhiteLine(false);
        sleep(100);
        turn(-90);

        approachBeacon();

        pushButton();

        // Go backwards slightly
        robot.rightMotor.setPower(-APPROACH_SPEED);
        robot.leftMotor.setPower(-APPROACH_SPEED);
        sleep(200);

        // Turn parallel to wall
        turn(0);
        sleep(2000);

        robot.leftMotor.setPower(APPROACH_SPEED);
        robot.rightMotor.setPower(APPROACH_SPEED);
        sleep(500);

        maintainDist();
        toWhiteLine(true);
        sleep(100);

        turn(-90);
        approachBeacon();
        pushButton();

        //Drives backward slightly
        robot.rightMotor.setPower(-APPROACH_SPEED);
        robot.leftMotor.setPower(-APPROACH_SPEED);
        sleep(200);

        turn(140);

        robot.rightMotor.setPower(APPROACH_SPEED);
        robot.leftMotor.setPower(APPROACH_SPEED);

        while ((opModeIsActive())){
            sleep(500);
            telemetry.update();
        }
    }

    void toWhiteLine(boolean wall) throws InterruptedException {
        // Start the robot moving forward, and then begin looking for a white line.
        if (!wall) {
            robot.leftMotor.setPower(APPROACH_SPEED * .8);
            robot.rightMotor.setPower(APPROACH_SPEED * .8);
        }

        // runIMU until the white line is seen OR the driver presses STOP;
        while (opModeIsActive() && (lightSensor.getLightDetected() < WHITE_THRESHOLD)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        telemetry.addData("Distance", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.update();

        // Stop all motors
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    double IMUheading() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    double getOpticalDistance(I2cDeviceSynchImpl rangeSensor) {
        return rangeSensor.read(0x04, 2)[1]  & 0xFF;
    }

    double getcmUltrasonic(I2cDeviceSynchImpl rangeSensor){
        return rangeSensor.read(0x04, 2)[0]  & 0xFF;
    }

    /*
    double getDistanceRange(DistanceUnit unit) {
        double measure =
        return unit.fromUnit(DistanceUnit.CM, )
    }
    */

    void turn(int turnAngle)
    {
        angleZ = IMUheading();

        if (turnAngle < angleZ) {
            robot.leftMotor.setPower(-APPROACH_SPEED * .6);
            robot.rightMotor.setPower(APPROACH_SPEED * .6);

            while (opModeIsActive() && (turnAngle < angleZ)) {

                // Display the light level while we are looking for the line
                angleZ = IMUheading();
                telemetry.addData("Angle", angleZ);
                telemetry.update();
                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            }
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
        }

        else if (turnAngle > angleZ) {
            robot.leftMotor.setPower(APPROACH_SPEED * .6);
            robot.rightMotor.setPower(APPROACH_SPEED * -.6);

            while (opModeIsActive() && (turnAngle > angleZ)) {

                // Display the light level while we are looking for the line
                angleZ = IMUheading();
                telemetry.addData("Angle", angleZ);
                telemetry.update();
                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            }
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
        }
    }

    void approachBeacon()
    {
        // Drive to set distance away, slow down, stop at set distance

        if (rangeSensor.getDistance(DistanceUnit.CM) > DIST * 3) {
            robot.leftMotor.setPower(APPROACH_SPEED);
            robot.rightMotor.setPower(APPROACH_SPEED);

            while (opModeIsActive() && rangeSensor.getDistance(DistanceUnit.CM) > DIST * 3) {

                telemetry.addData("Distance", rangeSensor.getDistance(DistanceUnit.CM));
                telemetry.update();

                idle();
            }
            //Momentarily stop
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            sleep(100);
            sleep(4000);
        }


        if (rangeSensor.getDistance(DistanceUnit.CM) > DIST) {
            robot.leftMotor.setPower(APPROACH_SPEED * .25);
            robot.rightMotor.setPower(APPROACH_SPEED * .25);
            while (opModeIsActive() && rangeSensor.getDistance(DistanceUnit.CM) > DIST) {

                telemetry.addData("Distance", rangeSensor.getDistance(DistanceUnit.CM));
                telemetry.update();

                idle();
            }
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            sleep(100);
            sleep(4000);
        }

        else {
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
        }
    }

    void pushButton() {
        // Pushes button, then straightens
        // REPLACE: Code to push button, use color sensor

        telemetry.log().add("in the push button method");

        leftColorSensor.enableLed(true);
        rightColorSensor.enableLed(true);

        int leftRed = leftColorSensor.red();
        int leftBlue = leftColorSensor.blue();
        int rightRed = rightColorSensor.red();
        int rightBlue = rightColorSensor.blue();

        while (!verify() && opModeIsActive()){
            telemetry.log().add("in the push button method while loop");

            if(leftRed > rightRed && !verify()){
                //write the code here to press the left button
                robot.leftMotor.setPower(0.3);
                robot.rightMotor.setPower(0.0);

                //wait three seconds
                verify();
            } else if(rightRed > leftRed && !verify()){
                //write the code here to press the right button
                robot.rightMotor.setPower(0.3);
                robot.leftMotor.setPower(0.0);
                verify();
            } else{
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
            }
        }

        telemetry.log().add("end of the push button method");

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    private boolean verify() {
        if(leftColorSensor.alpha() == 0 || rightColorSensor.alpha() == 0)
            return false;
        else if(leftColorSensor.alpha() == 255 || rightColorSensor.alpha() == 255)
            return false;

        if(Math.abs(leftColorSensor.red() - rightColorSensor.red()) < 2){
            return true;
        }

        return false;
    }

    void maintainDist() {

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        sideRange = sideRangeSensor.getDistance(DistanceUnit.CM);
        angleZ = IMUheading();
        telemetry.addData("Side Range: ", sideRangeSensor.getDistance(DistanceUnit.CM) );
        telemetry.addData("Angle", angleZ);
        telemetry.update();
        double distCorrect = SIDE_DIST - sideRange;

        //makes angle closer to 0
        robot.leftMotor.setPower(APPROACH_SPEED + angleZ/50 - distCorrect/50);
        robot.rightMotor.setPower(APPROACH_SPEED - angleZ/50 + distCorrect/50);
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
            newLeftTarget = robot.leftMotor.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            //robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            if (robot.leftMotor.getCurrentPosition() > newLeftTarget &&
                    robot.rightMotor.getCurrentPosition() > newRightTarget) {
                // keep looping while we are still active, and there is time left, and both motors are running.
                robot.leftMotor.setPower(speed);
                robot.rightMotor.setPower(speed);

                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        //(robot.leftMotor.isBusy() && robot.rightMotor.isBusy())
                        robot.leftMotor.getCurrentPosition() > newLeftTarget &&
                        robot.rightMotor.getCurrentPosition() > newRightTarget) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.leftMotor.getCurrentPosition(),
                            robot.rightMotor.getCurrentPosition());
                    telemetry.addData("Left motor busy", robot.leftMotor.isBusy());
                    telemetry.addData("Right motor busy", robot.rightMotor.isBusy());
                    telemetry.update();
                }
            }
            else if (robot.leftMotor.getCurrentPosition() < newLeftTarget &&
                    robot.rightMotor.getCurrentPosition() < newRightTarget) {
                // keep looping while we are still active, and there is time left, and both motors are running.
                robot.leftMotor.setPower(speed);
                robot.rightMotor.setPower(speed);

                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        //(robot.leftMotor.isBusy() && robot.rightMotor.isBusy())
                        robot.leftMotor.getCurrentPosition() < newLeftTarget &&
                        robot.rightMotor.getCurrentPosition() < newRightTarget) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.leftMotor.getCurrentPosition(),
                            robot.rightMotor.getCurrentPosition());
                    telemetry.addData("Left motor busy", robot.leftMotor.isBusy());
                    telemetry.addData("Right motor busy", robot.rightMotor.isBusy());
                    telemetry.update();
                }
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