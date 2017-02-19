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
package org.firstinspires.ftc.teamcode.Sam;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="PID Turn", group="Pushbot")
//@Disabled
public class DriveToBeaconsBlue extends LinearOpMode {

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
    public int ROTATION = 1220; // # of ticks
    double     COUNTS_PER_INCH         = ROTATION /
            (WHEEL_SIZE_IN * Math.PI);
    double DIST = 18;
    double SIDE_DIST = 30;
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
        // robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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


        // turn on LED of light sensor.
        lightSensor.enableLed(true);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to runIMU");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {

            // Display the light level while we are waiting to start
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            angleZ = IMUheading();
            telemetry.addData("Angle", angleZ);
            //telemetry.addData("verify", verify());
            telemetry.addData("leftColorSensor", leftColorSensor.argb());
            telemetry.addData("rightColorSensor", rightColorSensor.argb());
            telemetry.update();
            idle();
        }



        PIDturn(-90);



    }

    double IMUheading() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }


    void PIDturn(int turnAngle)
    {
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        angleZ = IMUheading();

        double Kp=0.0001;
        double Ki=0.0001;
        double Kd=0.0001;




        double previous_error=0;
        double integral=0, derivative=0, error=0;
        long dt=10;
        double angDiff = turnAngle-angleZ; //positive: turn left

        double output= Kp*angDiff + Ki*integral + Kd*derivative;
        double speed=output;

        if (Math.abs(angDiff) > 180) angDiff = angDiff % 180;

        if (angDiff < 0) { //turns right
            angDiff = turnAngle-angleZ; //positive: turn left
            error=Math.abs(angDiff);
            integral = integral + error*dt;//calculate integral of error
            derivative = (error - previous_error)/dt;//calculator derivative of data
            output= Kp*angDiff + Ki*integral + Kd*derivative;//summation of PID
            speed=output;
            previous_error=angDiff;

            robot.leftMotor.setPower(speed);
            robot.rightMotor.setPower(-speed);

            while (opModeIsActive() && (angDiff < 0)) {
                angleZ = IMUheading();
                angDiff = turnAngle-angleZ; //positive: turn left
                error=Math.abs(angDiff);
                integral = integral + error*dt;//calculate integral of error
                derivative = (error - previous_error)/dt;//calculator derivative of data
                output= Kp*angDiff + Ki*integral + Kd*derivative;//summation of PID
                speed=output;
                previous_error=angDiff;

                robot.leftMotor.setPower(speed);
                robot.rightMotor.setPower(-speed);


                telemetry.addData("Angle", angleZ);
                telemetry.addData("Speed: ", speed);
                telemetry.update();
                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
                sleep(dt);
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
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


}
