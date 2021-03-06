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

import org.firstinspires.ftc.teamcode.Shashank.statemachine.AllianceColor;

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

@Autonomous(name="Beacons Blue", group="Pushbot")
//@Disabled
public class DriveToBeaconsBlue extends LinearOpMode {

    //To change red to blue: negative angles, color sensors sense blue, right side range sensor

    /* Declare OpMode members. */
    AutonomousActions auto = new AutonomousActions(this);
    double FASTER_SPEED = .7;

    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        auto.init(hardwareMap, telemetry, AllianceColor.BLUE);

        telemetry.addData("verify", auto.verify()); //checks color sensors

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to runIMU");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {

            // Display the light level while we are waiting to start
            telemetry.addData("Light Level 1", auto.lightSensor1.getLightDetected());
            telemetry.addData("Light Level 2", auto.lightSensor2.getLightDetected());
            telemetry.addData("Front Ultrasonic", auto.getcmUltrasonic(auto.rangeSensor));
            auto.angleZ = auto.IMUheading();
            telemetry.addData("Angle", auto.angleZ);
            telemetry.addData("leftColorSensor", auto.leftColorSensor.argb());
            telemetry.addData("rightColorSensor", auto.rightColorSensor.argb());
            telemetry.update();
            idle();
        }

        auto.encoderDrive(0.3, 4, 4, 3);
        auto.turn(-45); //The robot uses the IMU to turn to 40 degrees
        auto.encoderDrive(FASTER_SPEED, 14, 14, 7);
        auto.toWhiteLine(false); //and then proceeds to the white line using encoders and a NXT light sensor

        sleep(100);
        auto.twoSensorLineFollow();
        auto.pushButton(); //The robot then uses two color sensors to push the blue side of the beacon, and verifies it press the correct side. If it didn't, then it will wait for 5 seconds and try again.
        auto.encoderDrive(auto.APPROACH_SPEED, auto.backup, auto.backup, 3); //The robot then moves backward using encoders
        boolean IMUreInit = false;
        if (auto.IMUheading() == 0) {
            auto.imu.initialize();
            IMUreInit = true;
        }
        if (!IMUreInit) {
            auto.turn(0); //and turns parallel to the beacon using the IMU
            sleep(200);
            if (Math.abs(auto.IMUheading()) > 7)
                auto.turn(0);
        } else
            auto.turn(90);
        auto.encoderDrive(FASTER_SPEED, auto.distanceBetweenBeacons, auto.distanceBetweenBeacons, 4);
        auto.toWhiteLine(true); //It advances to the next white line
        sleep(100);
        auto.twoSensorLineFollow();
        auto.pushButton(); //It uses two color sensors to push the blue side of the beacon, and verifies it press the correct side. If it didn't, then it will wait for 5 seconds and try again
        auto.encoderDrive(auto.APPROACH_SPEED, auto.backup - 4, auto.backup - 4, 3); //Then it will back up
        auto.turn(160);
        auto.encoderDrive(FASTER_SPEED, auto.capBallDistance, auto.capBallDistance, 5);
    }
}