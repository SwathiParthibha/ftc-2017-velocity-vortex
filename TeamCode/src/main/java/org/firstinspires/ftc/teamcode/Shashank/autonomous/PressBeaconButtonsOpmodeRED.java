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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

@Autonomous(name="PressBeaconButtonsOpmodeRED", group="Pushbot")
@Disabled
public class PressBeaconButtonsOpmodeRED extends LinearOpMode
{

    private ColorSensor leftColorSensor;
    private ColorSensor rightColorSensor;

    ModernRoboticsI2cGyro gyroSensor;   // Hardware Device Object
    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    LightSensor lightSensor;      // Primary LEGO Light sensor,
    ModernRoboticsI2cRangeSensor rangeSensor;
    // OpticalDistanceSensor   lightSensor;   // Alternative MR ODS sensor

    static final double WHITE_THRESHOLD = 0.3;  // spans between 0.1 - 0.5 from dark to light
    static final double APPROACH_SPEED = 0.5;
    double DIST = 11;

    @Override
    public void runOpMode() throws InterruptedException {

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
        //  lightSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");  // Alternative MR ODS sensor.

        // turn on LED of light sensor.
        lightSensor.enableLed(true);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        leftColorSensor  = hardwareMap.colorSensor.get("lcs");

        rightColorSensor = hardwareMap.colorSensor.get("rcs");

        I2cAddr i2cAddr = I2cAddr.create8bit(0x4c);
        leftColorSensor.setI2cAddress(i2cAddr);

        gyroSensor = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        gyroSensor.calibrate();

        while (!isStopRequested() && gyroSensor.isCalibrating())  {
            sleep(50);
            idle();
        }


        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {

            // Display the light level while we are waiting to start
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.addData("Distance", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
            idle();
        }

        int heading = gyroSensor.getHeading();
        int angleZ  = gyroSensor.getIntegratedZValue();

        telemetry.addData(">", "Press A & B to reset Heading.");
        telemetry.addData("0", "Heading %03d", heading);
        telemetry.addData("1", "Int. Ang. %03d", angleZ);
        telemetry.addData("power of left motor", robot.leftMotor.getPower());
        telemetry.addData("power of right motor", robot.rightMotor.getPower());
        telemetry.addData("cm in ultrasonic", rangeSensor.cmUltrasonic());
        telemetry.addData("cm in optical", rangeSensor.cmOptical());
        telemetry.addData("left", String.format("a=%d r=%d g=%d b=%d", leftColorSensor.alpha(), leftColorSensor.red(), leftColorSensor.green(), leftColorSensor.blue()));
        telemetry.addData("right", String.format("a=%d r=%d g=%d b=%d", rightColorSensor.alpha(), rightColorSensor.red(), rightColorSensor.green(), rightColorSensor.blue()));
        telemetry.addData("verify", verify());
        telemetry.update();

        //KEY EVENT
        //TODO: run to the first white line
        //first go to the white line
        toWhiteLine();

        turn(-50);

        telemetry.update();
        //MINOR EVENT
        //TODO: line up to beacon
        /*robot.leftMotor.setPower(-APPROACH_SPEED);
        sleep(750);
        robot.leftMotor.setPower(0);*/

        //MINOR EVENT
        //TODO: get close to beacon
        //approachBeacon();

        //waitForInSec(4);
        sleepThread(5000);

        telemetry.update();
        //KEY EVENT
        //TODO: push the correct button on the first beacon
        //push the red side of the beacons
        pushButton();

        telemetry.update();
        //MINOR EVENT
        //TODO: back away from beacon
        /*robot.rightMotor.setPower(-APPROACH_SPEED);
        robot.leftMotor.setPower(-APPROACH_SPEED);
        sleep(200);*/

        //MINOR EVENT
        //TODO: line up parallel to the wall
        /*robot.rightMotor.setPower(-APPROACH_SPEED);
        robot.leftMotor.setPower(APPROACH_SPEED);
        sleep(750); //REPLACE: Use gyro*/

        //waitForInSec(4);
        sleepThread(5000);

        telemetry.update();
        /*robot.leftMotor.setPower(APPROACH_SPEED);
        robot.rightMotor.setPower(APPROACH_SPEED);
        sleep(750);*/

        //KEY EVENT
        //TODO: run to the second white line
        toWhiteLine();

        telemetry.update();
        //MINOR EVENT
        //TODO: line up toward second beacon
        /*robot.rightMotor.setPower(APPROACH_SPEED);
        robot.leftMotor.setPower(-APPROACH_SPEED);
        sleep(750); //REPLACE: Use gyro*/

        //MINOR EVENT
        //TODO: get close to beacon
        //approachBeacon();

        //waitForInSec(4);

        sleepThread(5000);

        telemetry.update();
        //KEY EVENT
        //TODO: pusb the correct beacon button on the second beacon
        pushButton();
        telemetry.update();
    }

    private void turn(int degrees){
        while(gyroSensor.getIntegratedZValue() == degrees){
            if(degrees < 0){
                this.robot.leftMotor.setPower(0.3);
                this.robot.rightMotor.setPower(-0.3);
            } else {
                this.robot.leftMotor.setPower(-0.3);
                this.robot.rightMotor.setPower(0.3);
            }
        }

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    void toWhiteLine() throws InterruptedException {
        // Start the robot moving forward, and then begin looking for a white line.
        robot.leftMotor.setPower(APPROACH_SPEED);
        robot.rightMotor.setPower(APPROACH_SPEED);

        // run until the white line is seen OR the driver presses STOP;
        while (opModeIsActive() && (lightSensor.getLightDetected() < WHITE_THRESHOLD)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        telemetry.addData("Distance", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.update();

        // Stop all motors

        sleep(100);
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    void approachBeacon()
    {
        // Drive to set distance away, slow down, stop at set distance
        if (rangeSensor.getDistance(DistanceUnit.CM) > DIST * 2) {
            robot.leftMotor.setPower(APPROACH_SPEED);
            robot.rightMotor.setPower(APPROACH_SPEED);
        }

        while (opModeIsActive() && rangeSensor.getDistance(DistanceUnit.CM) > DIST  * 2) {

            telemetry.addData("Distance", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();

            idle();
        }

        //Momentarily stop
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        sleep(200);

        if (rangeSensor.getDistance(DistanceUnit.CM) > DIST) {
            robot.leftMotor.setPower(APPROACH_SPEED * .25);
            robot.rightMotor.setPower(APPROACH_SPEED * .25);
        }

        while (opModeIsActive() && rangeSensor.getDistance(DistanceUnit.CM) > DIST) {

            telemetry.addData("Distance", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();

            idle();
        }

        if (rangeSensor.getDistance(DistanceUnit.CM) > DIST) {
            robot.leftMotor.setPower(APPROACH_SPEED * .25);
            robot.rightMotor.setPower(APPROACH_SPEED * .25);
        }

        while (opModeIsActive() && rangeSensor.getDistance(DistanceUnit.CM) > DIST) {

            telemetry.addData("Distance", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();

            idle();
        }

        if (rangeSensor.getDistance(DistanceUnit.CM) > (DIST-3)) {
            robot.leftMotor.setPower(APPROACH_SPEED * .1);
            robot.rightMotor.setPower(APPROACH_SPEED * .1);
        }

        while (opModeIsActive() && rangeSensor.getDistance(DistanceUnit.CM) > (DIST-3)) {

            telemetry.addData("Distance", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();

            idle();
        }

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    void pushButton() {

        telemetry.update();

        int leftRed = 0;
        int rightRed = 0;

        double savedTime = this.time;
        int count = 20;
        while(!verify() && count > 0) {
            leftRed = leftColorSensor.red();
            rightRed = rightColorSensor.red();

            if(leftRed > rightRed && !verify()){
                //write the code here to press the left button
                robot.leftMotor.setPower(0.3);
                robot.rightMotor.setPower(0.0);
            } else if(rightRed > leftRed && !verify()){
                //write the code here to press the right button
                robot.rightMotor.setPower(0.3);
                robot.leftMotor.setPower(0.0);
                verify();
            } else{
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
            }

            telemetry.update();
        }

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        telemetry.update();
    }

    private void waitForInSec(int timeToWait) {
        while(timeToWait >= 0){
            telemetry.update();
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            sleepThread(1000);
            timeToWait--;
        }
    }

    private void sleepThread(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private boolean verify() {
        if(leftColorSensor.argb() == 0 || rightColorSensor.argb() == 0)
            return false;

        if(leftColorSensor.argb() == 255 || rightColorSensor.argb() == 255)
            return false;

        if(Math.abs(leftColorSensor.red() - rightColorSensor.red()) < 4){
            return true;
        }

        return false;
    }
}