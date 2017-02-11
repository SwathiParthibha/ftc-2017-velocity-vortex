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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@Autonomous(name="Push Button With Timeout", group="Pushbot")
//@Disabled
public class DriveToBeaconsBlue1 extends LinearOpMode {

    //To change red to blue: negative angles, color sensors sense blue, right side range sensor

    /* Declare OpMode members. */
    AutonomousActions auto = new AutonomousActions(this);

    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        auto.init(hardwareMap, telemetry);
        auto.runOpMode();

        //telemetry.addData("verifyBlue", auto.verifyBlue()); //checks color sensors

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to runIMU");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {

            // Display the light level while we are waiting to start
            telemetry.addData("Light Level", auto.lightSensor.getLightDetected());
            telemetry.addData("Front Ultrasonic", auto.getcmUltrasonic(auto.rangeSensor));
            auto.angleZ = auto.IMUheading();
            telemetry.addData("Side Ultrasonic", auto.getcmUltrasonic(auto.sideRangeSensor));
            telemetry.addData("Angle", auto.angleZ);
            //telemetry.addData("verifyBlue", verifyBlue());
            telemetry.addData("leftColorSensor", auto.leftColorSensor.argb());
            telemetry.addData("rightColorSensor", auto.rightColorSensor.argb());
            telemetry.update();
            idle();
        }

        pushBlueButton();
    }

    void pushBlueButton() throws InterruptedException {

        telemetry.log().add("in the push button method");

        telemetry.update();
        auto.leftColorSensor.enableLed(true);
        auto.rightColorSensor.enableLed(true);

        telemetry.update();
        int leftBlue = auto.leftColorSensor.blue();
        int rightBlue = auto.rightColorSensor.blue();

        ElapsedTime time = new ElapsedTime();
        time.reset();

        boolean wrongColor;

        do{
            telemetry.addData("Time", time.seconds());
            telemetry.log().add("in the push button method while loop");
            telemetry.addData("Left blue: ", auto.leftColorSensor.blue());
            telemetry.addData("Right blue: ", auto.rightColorSensor.blue());

            telemetry.update();

            wrongColor = false;

            if(auto.leftColorSensor.blue() > auto.rightColorSensor.blue()){// && !verifyBlue()){
                //write the code here to press the left button
                telemetry.log().add("left is blue");
                telemetry.update();

                auto.leftMotor.setPower(auto.APPROACH_SPEED); //motors seem to work in reverse
                auto.rightMotor.setPower(0);
            } else if(auto.rightColorSensor.blue() > auto.leftColorSensor.blue()) {// && !verifyBlue()){
                //write the code here to press the right button
                telemetry.log().add("right is blue");
                telemetry.update();

                auto.rightMotor.setPower(auto.APPROACH_SPEED); //motors seem to work in reverse
                auto.leftMotor.setPower(0);
            } else if(auto.leftColorSensor.red() > auto.leftColorSensor.blue() &&
                    auto.rightColorSensor.red() > auto.rightColorSensor.blue()){
                //red button has been pressed
                telemetry.log().add("beacon is red");
                telemetry.update();

                //sleep(4000); // wait 5 seconds total
                auto.leftMotor.setPower(auto.APPROACH_SPEED);
                auto.rightMotor.setPower(auto.APPROACH_SPEED);

                wrongColor = true;

            } else if(auto.getcmUltrasonic(auto.rangeSensor) > 8) {
                auto.encoderDrive(auto.APPROACH_SPEED, 1, 1, 1);
            } else{
                auto.leftMotor.setPower(0);
                auto.rightMotor.setPower(0);
                telemetry.log().add("blue is not detected");
                telemetry.update();
                break;
            }
            telemetry.update();
            sleep(1500);
            auto.leftMotor.setPower(0);
            auto.rightMotor.setPower(0);

            //leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            auto.leftMotor.setPower(-auto.APPROACH_SPEED * .8);
            auto.rightMotor.setPower(-auto.APPROACH_SPEED * .8);
            sleep(100);
            auto.leftMotor.setPower(0);
            auto.rightMotor.setPower(0);

            telemetry.addData("Left blue: ", auto.leftColorSensor.blue());
            telemetry.addData("Right blue: ", auto.rightColorSensor.blue());
            telemetry.update();

            //leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            //rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } while (opModeIsActive() && !auto.verifyBlue()
                && (time.seconds() < 8 || wrongColor));

        telemetry.log().add("end of the push button method");

        auto.leftMotor.setPower(0);
        auto.rightMotor.setPower(0);
    }
}