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
package org.firstinspires.ftc.teamcode.Shashank.autonomous;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Mrinali.OldAutonomous.HardwarePushbot;

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

@Autonomous(name="Beacons Autonomous Blue Shashank No Shooting", group="Pushbot")
@Disabled
public class DriveToBeaconsBlueNoShooting extends LinearOpMode {

    //To change red to blue: negative angles, color sensors sense blue, right side range sensor
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX ShooterMotor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 5.0 ;     // For figuring circumference for the monster tires
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
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

    // get a reference to a Modern Robotics GyroSensor object.

    static final double WHITE_THRESHOLD = 0.3;  // spans between 0.1 - 0.5 from dark to light
    static final double APPROACH_SPEED = 0.7;
    double DIST = 6;
    double SIDE_DIST = 10;
    byte[] rangeSensorCache;
    byte[] sideRangeSensorCache;
    I2cDevice rangeA;
    I2cDevice rangeB;

    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;

    private DcMotor shooter;
    private DcMotor scooper;

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

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // get a reference to our Light Sensor object.
        lightSensor = hardwareMap.lightSensor.get("light sensor");
        rangeA = hardwareMap.i2cDevice.get("r1");// Primary LEGO Light Sensor
        rangeSensor = new I2cDeviceSynchImpl(rangeA, I2cAddr.create8bit(0x2a), false);
        rangeB = hardwareMap.i2cDevice.get("r2");// Primary LEGO Light Sensor
        sideRangeSensor = new I2cDeviceSynchImpl(rangeB, I2cAddr.create8bit(0x38), false);

        leftMotor = robot.leftMotor;
        rightMotor = robot.rightMotor;

        scooper = this.hardwareMap.dcMotor.get("scooper");
        shooter = this.hardwareMap.dcMotor.get("shooter");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        rangeSensor.engage();
        sideRangeSensor.engage();

        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);

        leftColorSensor  = hardwareMap.colorSensor.get("lcs");
        rightColorSensor = hardwareMap.colorSensor.get("rcs");
        I2cAddr i2cAddr = I2cAddr.create8bit(0x4c);
        leftColorSensor.setI2cAddress(i2cAddr);

        // turn on LED of light sensor.
        lightSensor.enableLed(true);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to runIMU");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {

            // Display the light level while we are waiting to start
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.addData("Distance", getOpticalDistance(rangeSensor));
            telemetry.addData("Distance sonic", getcmUltrasonic(rangeSensor));

         angleZ = IMUheading();
            telemetry.addData("Side Range Sensor", getOpticalDistance(sideRangeSensor));
            telemetry.addData("Side Range Sensor sonic", getcmUltrasonic(sideRangeSensor));
            telemetry.addData("Angle", angleZ);
            telemetry.addData("Angle not formatted", imu.getAngularOrientation().firstAngle);
            telemetry.addData("verify", verify());
            telemetry.addData("leftColorSensor", leftColorSensor.argb());
            telemetry.addData("rightColorSensor", rightColorSensor.argb());
            telemetry.update();
            idle();
        }

        /*telemetry.log().add("Now turning -90");
        turn(-90);
        sleep(2000);
        telemetry.log().add("Now turning 0");
        turn(0);
        sleep(2000);
        telemetry.log().add("Now turning 90");
        turn(90);
        sleep(2000);
        telemetry.log().add("Now turning 110");
        turn(110);
        sleep(2000);*/
        sleep(100);
        turn(-40);
        sleep(100);
        telemetry.log().add("Finished turning to -40 degrees");
        toWhiteLine(false);
        telemetry.update();
        sleep(100);
        robot.leftMotor.setPower(0.3);
        robot.rightMotor.setPower(0.3);
        sleep(50);
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        turn(-90);
        sleep(200);
        telemetry.update();
        approachBeacon();
        sleep(100);
        telemetry.update();
        pushButton();
        telemetry.update();

        telemetry.log().add("Finished pressing first button");
        sleep(100);

        // Go backwards slightly
        robot.rightMotor.setPower(-APPROACH_SPEED);
        robot.leftMotor.setPower(-APPROACH_SPEED);
        sleep(200);
        telemetry.update();

        telemetry.log().add("Backed up");
        robot.rightMotor.setPower(0);
        robot.leftMotor.setPower(0);

        // Turn parallel to wall
        turn(0);
        sleep(100);
        telemetry.update();
        telemetry.log().add("Turn parallel to the wall");
        sleep(100);

        robot.leftMotor.setPower(0.3);
        robot.rightMotor.setPower(0.3);
        telemetry.update();
        sleep(500);
        telemetry.update();

        telemetry.log().add("Run parallel to the wall");

        //maintainDist();
        telemetry.update();
        toWhiteLine(false);
        telemetry.update();
        sleep(200);
        telemetry.log().add("Turn to second beacon");
        turn(-90);
        approachBeacon();
        sleep(100);
        pushButton();

        telemetry.log().add("Drives backward slightly");
        //Drives backward slightly
        robot.rightMotor.setPower(-APPROACH_SPEED);
        robot.leftMotor.setPower(-APPROACH_SPEED);
        sleep(500);

        turn(-40);
        sleep(200);

        robot.rightMotor.setPower(-APPROACH_SPEED);
        robot.leftMotor.setPower(-APPROACH_SPEED);

        sleep(1700);

        robot.rightMotor.setPower(0);
        robot.leftMotor.setPower(0);

        while ((opModeIsActive())){
            sleep(500);
            telemetry.update();
        }
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
            robot.leftMotor.setPower(APPROACH_SPEED*0.7);
            robot.rightMotor.setPower(APPROACH_SPEED*0.7);
        }

        // runIMU until the white line is seen OR the driver presses STOP;
        while (opModeIsActive() && (lightSensor.getLightDetected() < WHITE_THRESHOLD)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        telemetry.addData("Distance", getcmUltrasonic(rangeSensor));
        telemetry.update();

        // Stop all motors
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    double IMUheading() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    void turn(int turnAngle)
    {
        angleZ = IMUheading();

        telemetry.update();

        if (turnAngle < angleZ) {
            robot.leftMotor.setPower(-APPROACH_SPEED*0.85);
            robot.rightMotor.setPower(APPROACH_SPEED*0.85);

            telemetry.log().add("trunAngle is less than anglez");
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
            robot.leftMotor.setPower(APPROACH_SPEED*0.85);
            robot.rightMotor.setPower(-APPROACH_SPEED*0.85);

            while (opModeIsActive() && (turnAngle > angleZ)) {

                telemetry.log().add("trunAngle is more than anglez");
                // Display the light level while we are looking for the line
                angleZ = IMUheading();
                telemetry.addData("Angle", angleZ);
                telemetry.update();
                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            }
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
        }

        telemetry.log().add("difference: " + (turnAngle-angleZ));
        sleep(150);
        if((Math.abs(turnAngle - angleZ)) > 0.4)
            turn(turnAngle);

        return;
    }

    void approachBeacon()
    {

        //Momentarily stop
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        sleep(200);

        telemetry.addData("Distance", getcmUltrasonic(rangeSensor));
        telemetry.update();

        telemetry.log().add("30 cm");
        while (opModeIsActive() && getcmUltrasonic(rangeSensor) > 11) {

            telemetry.log().add("in 30 cm loop");
            robot.leftMotor.setPower(0.4);
            robot.rightMotor.setPower(0.4);

            sleep(150);

            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            telemetry.addData("Distance", getcmUltrasonic(rangeSensor));
            telemetry.update();

            idle();
        }

        telemetry.log().add("end 25 cm");
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        sleep(200);

        telemetry.log().add("15 cm");
        while (opModeIsActive() && getcmUltrasonic(rangeSensor) > 8) {

            telemetry.log().add("in 15 cm loop");
            robot.leftMotor.setPower(0.5);
            robot.rightMotor.setPower(0.5);

            sleep(5);

            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            telemetry.addData("Distance", getcmUltrasonic(rangeSensor));
            telemetry.update();

            idle();
        }

        telemetry.log().add("end 15 cm");
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        sleep(200);

        telemetry.addData("Distance", getcmUltrasonic(rangeSensor));
        telemetry.update();
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    void pushButton() {
        // Pushes button, then straightens
        // REPLACE: Code to push button, use color sensor

        telemetry.log().add("in the push button method");

        telemetry.update();
        leftColorSensor.enableLed(true);
        rightColorSensor.enableLed(true);

        telemetry.update();
        int leftRed = leftColorSensor.red();
        int rightRed = rightColorSensor.red();

        double startTime = this.time;

        telemetry.addData("saved time", startTime);
        telemetry.addData("current time", this.time);

        do{
            telemetry.log().add("in the push button method while loop");

            telemetry.addData("saved time", startTime);
            telemetry.addData("current time", this.time);
            telemetry.update();

            if((this.time - startTime) > 5)
                break;

            if(leftColorSensor.blue() > leftColorSensor.red()
                    && rightColorSensor.red() > rightColorSensor.blue()
                    && !verify()){
                //write the code here to press the left button
                robot.leftMotor.setPower(0.5);
                robot.rightMotor.setPower(-0.2);

                //wait three seconds
                verify();
                telemetry.log().add("left is red "+ verify());
                telemetry.update();
            } else if(leftColorSensor.blue() < leftColorSensor.red()
                    && rightColorSensor.red() < rightColorSensor.blue()
                    && !verify()){
                //write the code here to press the right button
                robot.rightMotor.setPower(0.5);
                robot.leftMotor.setPower(-0.2);
                verify();
                telemetry.log().add("right is red "+ verify());
                telemetry.update();
            } else{
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
                telemetry.log().add("red is not detected "+ verify());
                telemetry.update();
            }
            telemetry.update();
        } while  (!verify() && opModeIsActive());

        telemetry.log().add("end of the push button method");

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    private boolean verify() {
        if(leftColorSensor.alpha() == 255 || rightColorSensor.alpha() == 255)
            throw new RuntimeException("Color Sensor problems");

        if(leftColorSensor.alpha() < 1 && rightColorSensor.alpha() < 1){
            return false;
        }

        if(Math.abs(leftColorSensor.blue() - rightColorSensor.blue()) <= 2){
            return true;
        }

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
        double distCorrect = SIDE_DIST - sideRange;

        //makes angle closer to 0
        robot.leftMotor.setPower(APPROACH_SPEED + angleZ/50 + distCorrect/50);
        robot.rightMotor.setPower(APPROACH_SPEED - angleZ/50 - distCorrect/50);

    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double leftPower = speed;
            double rightPower = speed;
            if(leftInches < 0){
                leftPower = leftPower*-1;
            } else if (rightInches < 0){
                rightPower = rightPower*-1;
            }

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }
}