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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Sam.shooter.MotorFactory;
import org.firstinspires.ftc.teamcode.Sam.shooter.beans.ShooterMotor;
import org.firstinspires.ftc.teamcode.Sam.shooter.power.PowerManager;
import org.firstinspires.ftc.teamcode.Sam.shooter.util.Constants;

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

public class ExperimentAutonomousActions extends LinearOpMode {

    //To change red to blue: negative angles, color sensors sense blue, right side range sensor

    /* Declare OpMode members. */
    //HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    LinearOpMode opMode;
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor shooter1;
    public DcMotor shooter2;
    private PowerManager leftShooterPowerMgr;
    private PowerManager rightShooterPowerMgr;
    private Servo leftArm;
    private Servo rightArm;
    private Servo capArm;
    private boolean state;
    public DcMotor scooper;
    public LightSensor lightSensor;      // Primary LEGO Light sensor,
    public I2cDeviceSynchImpl rangeSensor;
    public I2cDeviceSynchImpl sideRangeSensor;
    double sideRange;
    //ModernRoboticsI2cGyro gyro;   // Hardware Device Object
    public ColorSensor leftColorSensor;
    public ColorSensor rightColorSensor;
    BNO055IMU imu;
    Orientation angles;

    // OpticalDistanceSensor   lightSensor;   // Alternative MR ODS sensor
    public double angleZ = 0;

    static final double WHITE_THRESHOLD = 0.3;  // spans between 0.1 - 0.5 from dark to light
    public static final double APPROACH_SPEED = 0.5;
    double TURN_POWER_1 = .2;
    double TURN_POWER_2 = .05;
    double WHEEL_SIZE_IN = 4;
    public int ROTATION = 1220; // # of ticks for 40-1 gear ratio
    static final double DRIVE_GEAR_REDUCTION = 1.5;     // This is < 1.0 if geared UP
    double GEAR_RATIO = 40;
    double COUNTS_PER_INCH = (ROTATION * DRIVE_GEAR_REDUCTION) /
            (WHEEL_SIZE_IN * Math.PI) * (40 / GEAR_RATIO);
    double DIST = 18;
    double SIDE_DIST = 30;
    public double backup = -2;
    double overLine1 = 2;
    double overLine2 = 2;
    private final double LEFT_IN_VAL = 0.56;
    private final double RIGHT_IN_VAL = 0.34;
    private final double LEFT_OUT_VAL = 0.12;
    private final double RIGHT_OUT_VAL = 0.76;
    private final double SERVO_ADJUSTMENT_VAL_LEFT = (Math.abs(LEFT_IN_VAL - LEFT_OUT_VAL) / 14);
    private final double SERVO_ADJUSTMENT_VAL_RIGHT = (Math.abs(RIGHT_IN_VAL - RIGHT_OUT_VAL) / 14);
    private final double LEFT_START_VAL = (LEFT_IN_VAL + 2*LEFT_OUT_VAL) /3;
    private final double RIGHT_START_VAL = (RIGHT_IN_VAL + 2*RIGHT_OUT_VAL) /3;
    double leftServoPos = 0;
    double rightServoPos = 1.0;
    double capServoPos = 1.0;
    byte[] rangeSensorCache;
    byte[] sideRangeSensorCache;
    I2cDevice rangeA;
    I2cDevice rangeB;
    double initialTilt;
    private boolean USE_TELEMETRY = false;

    int whiteCounter = 0;
    int blackCounter = 0;

    public ExperimentAutonomousActions(LinearOpMode anOpMode) {
        opMode = anOpMode;
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public double startShootingtime = 0;
    public double prevTime = 0;

    public void init(HardwareMap hardwareMap, Telemetry telem) throws InterruptedException {

        // Define and Initialize Motors
        leftMotor = hardwareMap.dcMotor.get("l");
        rightMotor = hardwareMap.dcMotor.get("r");

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1 = hardwareMap.dcMotor.get("shooter1");
        shooter2 = hardwareMap.dcMotor.get("shooter2");

        MotorFactory motorFactory = MotorFactory.getInstance();
        ShooterMotor leftShooter = new ShooterMotor();
        leftShooter.setName(Constants.MOTORNAME.LEFT_SHOOTER);
        motorFactory.addMotor(leftShooter);

        ShooterMotor rightShooter = new ShooterMotor();
        rightShooter.setName(Constants.MOTORNAME.RIGHT_SHOOTER);
        motorFactory.addMotor(rightShooter);

        leftShooterPowerMgr = new PowerManager(Constants.MOTORNAME.LEFT_SHOOTER, shooter1, opMode);
        rightShooterPowerMgr = new PowerManager(Constants.MOTORNAME.RIGHT_SHOOTER, shooter2, opMode);

        scooper = hardwareMap.dcMotor.get("scooper");

        leftArm = hardwareMap.servo.get("leftservo");
        rightArm = hardwareMap.servo.get("rightservo");
        leftServoPos = LEFT_START_VAL;
        rightServoPos = RIGHT_START_VAL;
        leftArm.setPosition(leftServoPos);
        rightArm.setPosition(rightServoPos);

        capArm = hardwareMap.servo.get("capArm");
        capArm.setPosition(capServoPos);

        state = false;

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry = telem;

        // get a reference to our Light Sensor object.
        lightSensor = hardwareMap.lightSensor.get("light sensor");
        rangeA = hardwareMap.i2cDevice.get("range sensor");// Primary LEGO Light Sensor
        rangeSensor = new I2cDeviceSynchImpl(rangeA, I2cAddr.create8bit(0x2a), false);
        rangeA = hardwareMap.i2cDevice.get("r side range");// Primary LEGO Light Sensor
        sideRangeSensor = new I2cDeviceSynchImpl(rangeA, I2cAddr.create8bit(0x28), false);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        initialTilt = frontTilt();

        rangeSensor.engage();
        sideRangeSensor.engage();

        //angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        //origAngle = angles.firstAngle;

        leftColorSensor = hardwareMap.colorSensor.get("lcs");

        rightColorSensor = hardwareMap.colorSensor.get("rcs");
        I2cAddr i2cAddr = I2cAddr.create8bit(0x4c);
        rightColorSensor.setI2cAddress(i2cAddr);

        lightSensor.enableLed(true);
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

    public void toWhiteLine(boolean wall) throws InterruptedException {
        // Start the robot moving forward, and then begin looking for a white line.
        /*
        if (!wall) {
            leftMotor.setPower(APPROACH_SPEED * .4);
            rightMotor.setPower(APPROACH_SPEED * .4);
        }
        */

        while (opMode.opModeIsActive() && lightSensor.getLightDetected() < WHITE_THRESHOLD) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();
            idle();

            if (imu.getLinearAcceleration().zAccel < 0.2) {
                leftMotor.setPower(APPROACH_SPEED * .4);
                rightMotor.setPower(APPROACH_SPEED * .4);
            }

            whiteCounter = 1;
        }

        // Stop all motors
        stopRobot();

        if (!wall) {
            encoderDrive(APPROACH_SPEED * .4, .75, .75, 1);
        } else
            encoderDrive(APPROACH_SPEED * .4, 1, 1, 2);
    }

    public void toWhiteLineCheckTilt(boolean wall, String color) throws InterruptedException {
        // Start the robot moving forward, and then begin looking for a white line.
        /*
        if (!wall) {
            leftMotor.setPower(APPROACH_SPEED * .4);
            rightMotor.setPower(APPROACH_SPEED * .4);
        }
        */

        while (opMode.opModeIsActive() && lightSensor.getLightDetected() < WHITE_THRESHOLD) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();
            idle();

            if (imu.getLinearAcceleration().zAccel < 0.2) {
                leftMotor.setPower(APPROACH_SPEED * .4);
                rightMotor.setPower(APPROACH_SPEED * .4);
            }

            if (frontTilt() > initialTilt + 2) {
                telemetry.addData("tilt", frontTilt());
                encoderDrive(APPROACH_SPEED * .6, -.5, -.5, 1);
                if (color == "blue")
                    spinLeft();
                else if (color == "red")
                    spinRight();
            }
        }

        // Stop all motors
        stopRobot();

        if (!wall) {
            encoderDrive(APPROACH_SPEED * .4, .75, .75, 1);
        } else
            encoderDrive(APPROACH_SPEED * .4, 1, 1, 2);
    }

    void spinRight() throws InterruptedException {
        angleZ = IMUheading();
        leftMotor.setPower(APPROACH_SPEED); //spinRight right
        rightMotor.setPower(-APPROACH_SPEED);
        while (opModeIsActive() && IMUheading() > -180) {
            telemetry.addData("Angle", IMUheading());
        }
        while (opModeIsActive() && IMUheading() > 90) {
            telemetry.addData("Angle", IMUheading());
        }
        turn(0);
        leftMotor.setPower(0); //spin right
        rightMotor.setPower(0);
    }

    void spinLeft() throws InterruptedException {
        angleZ = IMUheading();
        leftMotor.setPower(-APPROACH_SPEED); //spinRight right
        rightMotor.setPower(APPROACH_SPEED);
        while (opModeIsActive() && IMUheading() < 180) {
            telemetry.addData("Angle", IMUheading());
        }
        while (opModeIsActive() && IMUheading() < -90) {
            telemetry.addData("Angle", IMUheading());
        }
        turn(0);
        leftMotor.setPower(0); //spin left
        rightMotor.setPower(0);
    }

    public void turn(int turnAngle) throws InterruptedException {

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int leftPos = leftMotor.getCurrentPosition();
        int rightPos = rightMotor.getCurrentPosition();

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

                leftMotor.setPower(turnPower(angDiff));
                rightMotor.setPower(-turnPower(angDiff));

                if (leftMotor.getCurrentPosition() - 100 > leftPos
                        && rightMotor.getCurrentPosition() + 100 < rightPos
                        && IMUheading() == startAngle) {
                    resetIMuandPos(leftPos, rightPos);
                }

                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            }
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        } else if (angDiff > 0) {
            ; //turns left
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

                leftMotor.setPower(-turnPower(angDiff));
                rightMotor.setPower(turnPower(angDiff));

                if (leftMotor.getCurrentPosition() + 100 < leftPos
                        && rightMotor.getCurrentPosition() - 100 > rightPos
                        && IMUheading() == startAngle) {
                    resetIMuandPos(leftPos, rightPos);
                }

                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            }
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    double turnPower(double difference) {
        if (Math.abs(difference) < 20) {
            return 0.05;
        } else if (Math.abs(difference) < 45) {
            return 0.1;
        } else if (Math.abs(difference) < 90) {
            return 0.3;
        } else return 0.5;
    }

    void resetIMuandPos(int left, int right) throws InterruptedException { //resets IMU to 0 at starting position of turn

        telemetry.addLine("IMU Problem");
        telemetry.update();

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setTargetPosition(left);
        rightMotor.setTargetPosition(right);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightMotor.setPower(.2);
        leftMotor.setPower(.2);

        while (opMode.opModeIsActive() &&
                (leftMotor.isBusy() && rightMotor.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d", left, right);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    leftMotor.getCurrentPosition(),
                    rightMotor.getCurrentPosition());
            telemetry.update();

            idle();
        }

        rightMotor.setPower(0);
        leftMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Initializing IMU");
        telemetry.update();
        imu.initialize();
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

    public void followLineBlueSide() throws InterruptedException {
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addLine("Following Line");
        leftMotor.setPower(.2);
        rightMotor.setPower(-.2);
        while (opMode.opModeIsActive() && lightSensor.getLightDetected() < WHITE_THRESHOLD) {
            telemetry.addData("Light", lightSensor.getLightDetected());
            idle();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        while (opMode.opModeIsActive() && getcmUltrasonic(rangeSensor) > 11) {
            telemetry.addData("Front range", getcmUltrasonic(rangeSensor));
            telemetry.addData("Light", lightSensor.getLightDetected());
            if (lightSensor.getLightDetected() > WHITE_THRESHOLD) {
                telemetry.addLine("Moving right");
                leftMotor.setPower(0.2);
                rightMotor.setPower(0);
            } else {
                telemetry.addLine("Moving left");
                leftMotor.setPower(0);
                rightMotor.setPower(0.2);
            }
            telemetry.update();

            idle();
        }
        stopRobot();
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void followLineRedSide() throws InterruptedException {
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addLine("Following Line");
        leftMotor.setPower(-.2);
        rightMotor.setPower(.2);
        while (opMode.opModeIsActive() && lightSensor.getLightDetected() < WHITE_THRESHOLD) {
            telemetry.addData("Light", lightSensor.getLightDetected());
            idle();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        while (opMode.opModeIsActive() && getcmUltrasonic(rangeSensor) > 11) {
            telemetry.addData("Front range", getcmUltrasonic(rangeSensor));
            telemetry.addData("Light", lightSensor.getLightDetected());
            if (lightSensor.getLightDetected() > WHITE_THRESHOLD) {
                telemetry.addLine("Moving left");
                leftMotor.setPower(0);
                rightMotor.setPower(0.2);
            } else {
                telemetry.addLine("Moving right");
                leftMotor.setPower(0.2);
                rightMotor.setPower(0);
            }
            telemetry.update();

            idle();
        }
        stopRobot();
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void followLineBlueSide1() throws InterruptedException {
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addLine("Following Line");
        leftMotor.setPower(.2);
        rightMotor.setPower(-.2);
        while (opMode.opModeIsActive() && lightSensor.getLightDetected() < WHITE_THRESHOLD) {
            telemetry.addData("Light", lightSensor.getLightDetected());
            idle();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        while (opMode.opModeIsActive() && getcmUltrasonic(rangeSensor) > 11) {
            telemetry.addData("Front range", getcmUltrasonic(rangeSensor));
            telemetry.addData("Light", lightSensor.getLightDetected());
            if (lightSensor.getLightDetected() > WHITE_THRESHOLD) {
                whiteCounter++;
                telemetry.addLine("Moving right");
                leftMotor.setPower(0.2);
                rightMotor.setPower(0);
            }
            else if (lightSensor.getLightDetected() < WHITE_THRESHOLD &&
                    (whiteCounter - blackCounter > 0 && whiteCounter - blackCounter < 4)){
                blackCounter++;
                telemetry.addLine("Moving left");
                leftMotor.setPower(0);
                rightMotor.setPower(0.2);
            }
            else if (whiteCounter - blackCounter < 1)
            {
                whiteCounter++;
                long start = System.currentTimeMillis();
                while (System.currentTimeMillis()- start < 500) {
                    telemetry.addLine("Moving right");
                    leftMotor.setPower(0.2);
                    rightMotor.setPower(0);
                }
            }
            else
            {
                blackCounter++;
                long start = System.currentTimeMillis();
                while (System.currentTimeMillis()- start < 500) {
                    telemetry.addLine("Moving right");
                    leftMotor.setPower(0.2);
                    rightMotor.setPower(0);
                }
            }
            telemetry.update();

            idle();
        }
        stopRobot();
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void pushBlueButton() throws InterruptedException {

        telemetry.log().add("in the push button method");

        telemetry.update();
        leftColorSensor.enableLed(true);
        rightColorSensor.enableLed(true);

        telemetry.update();
        int leftBlue = leftColorSensor.blue();
        int rightBlue = rightColorSensor.blue();

        ElapsedTime time = new ElapsedTime();
        time.reset();

        boolean wrongColor;

        do {
            telemetry.addData("Time", time.seconds());
            telemetry.log().add("in the push button method while loop");
            telemetry.addData("Left blue: ", leftColorSensor.blue());
            telemetry.addData("Right blue: ", rightColorSensor.blue());

            telemetry.update();

            wrongColor = false;

            if (leftColorSensor.blue() > rightColorSensor.blue()) {// && !verifyBlue()){
                //write the code here to press the left button
                telemetry.log().add("left is blue");
                telemetry.update();

                leftMotor.setPower(APPROACH_SPEED * .6); //motors seem to work in reverse
                rightMotor.setPower(0);
            } else if (rightColorSensor.blue() > leftColorSensor.blue()) {// && !verifyBlue()){
                //write the code here to press the right button
                telemetry.log().add("right is blue");
                telemetry.update();

                rightMotor.setPower(APPROACH_SPEED * .6); //motors seem to work in reverse
                leftMotor.setPower(0);
            } else if (leftColorSensor.red() > leftColorSensor.blue() &&
                    rightColorSensor.red() > rightColorSensor.blue()) {
                //red button has been pressed
                telemetry.log().add("beacon is red");
                telemetry.update();

                //sleep(4000); // wait 5 seconds total
                leftMotor.setPower(APPROACH_SPEED * .6);
                rightMotor.setPower(APPROACH_SPEED * .6);

                wrongColor = true;
            } else if (getcmUltrasonic(rangeSensor) > 8) {
                telemetry.log().add("too far");
                encoderDrive(APPROACH_SPEED * .6, 1, 1, 1);
            } else {
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
            sleep(100);
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            telemetry.addData("Left blue: ", leftColorSensor.blue());
            telemetry.addData("Right blue: ", rightColorSensor.blue());
            telemetry.update();

            idle();
            //leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            //rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } while (opMode.opModeIsActive() && !verifyBlue()
                && (time.seconds() < 4 || wrongColor));

        telemetry.log().add("end of the push button method");

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void pushRedButton() throws InterruptedException {

        telemetry.log().add("in the push button method");

        telemetry.update();
        leftColorSensor.enableLed(true);
        rightColorSensor.enableLed(true);

        telemetry.update();
        int leftBlue = leftColorSensor.blue();
        int rightBlue = rightColorSensor.blue();

        ElapsedTime time = new ElapsedTime();
        time.reset();

        boolean wrongColor;

        do {
            telemetry.log().add("in the push button method while loop");
            telemetry.addData("Left red: ", leftColorSensor.red());
            telemetry.addData("Right red: ", rightColorSensor.red());

            telemetry.update();

            wrongColor = false;

            if (leftColorSensor.red() > rightColorSensor.red()) {// && !verifyBlue()){
                //write the code here to press the left button
                telemetry.log().add("left is red");
                telemetry.update();

                leftMotor.setPower(APPROACH_SPEED * .6); //motors seem to work in reverse
                rightMotor.setPower(0);
            } else if (rightColorSensor.red() > leftColorSensor.red()) {// && !verifyBlue()){
                //write the code here to press the right button
                telemetry.log().add("right is red");
                telemetry.update();

                rightMotor.setPower(APPROACH_SPEED * .6); //motors seem to work in reverse
                leftMotor.setPower(0);
            } else if (leftColorSensor.blue() > leftColorSensor.red()
                    && rightColorSensor.blue() > rightColorSensor.red()) {
                //red button has been pressed
                telemetry.log().add("beacon is blue");
                telemetry.update();

                //sleep(4000); // wait 5 seconds total
                leftMotor.setPower(APPROACH_SPEED * .6);
                rightMotor.setPower(APPROACH_SPEED * .6);

                wrongColor = true;
            } else if (getcmUltrasonic(rangeSensor) > 8) {
                encoderDrive(APPROACH_SPEED * .6, 1, 1, 1);
            } else {
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

            idle();
        } while (opMode.opModeIsActive() && !verifyRed()
                && (time.seconds() < 4 || wrongColor));

        telemetry.log().add("end of the push button method");

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public boolean verifyBlue() {
        if (leftColorSensor.alpha() == 255 || rightColorSensor.alpha() == 255)
            throw new RuntimeException("Color Sensor problems");
        /*else if (leftColorSensor.red() == rightColorSensor.red()
                && leftColorSensor.blue() == rightColorSensor.blue()
                && leftColorSensor.red() > 2
                && rightColorSensor.red() > 2)
            throw new RuntimeException("Color Sensor problems");*/

        if (leftColorSensor.blue() > leftColorSensor.red() && rightColorSensor.blue() > rightColorSensor.red()) {
            telemetry.addLine("Beacon is blue");
            return true;
        }
        /*else if(Math.abs(leftColorSensor.blue() - rightColorSensor.blue()) < 2){
            return true;
        }*/
        telemetry.addLine("Beacon is red");
        return false;
    }

    public boolean verifyRed() {
        if (leftColorSensor.alpha() == 255 || rightColorSensor.alpha() == 255)
            throw new RuntimeException("Color Sensor problems");
        /*else if (leftColorSensor.red() == rightColorSensor.red()
                && leftColorSensor.blue() == rightColorSensor.blue()
                && leftColorSensor.red() > 2
                && rightColorSensor.red() > 2)
            throw new RuntimeException("Color Sensor problems");*/

        if (leftColorSensor.red() > leftColorSensor.blue() && rightColorSensor.red() > rightColorSensor.blue()) {
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
        leftMotor.setPower(APPROACH_SPEED * .6 + angleZ / 50 - distCorrect / 60);
        rightMotor.setPower(APPROACH_SPEED * .6 - angleZ / 50 + distCorrect / 60);

    }

    double frontTilt() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.thirdAngle));
    }

    public void stopRobot() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();
        int newLeftTarget;
        int newRightTarget;

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        newLeftTarget = leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newRightTarget = rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
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

            idle();
        }
        // Stop all motion;
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  sleep(250);   // optional pause after each move
    }

    public void encoderDriveCheckTilt(double speed,
                                      double leftInches, double rightInches,
                                      double timeoutS, String color) throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();
        int newLeftTarget;
        int newRightTarget;

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        newLeftTarget = leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newRightTarget = rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
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

            if (frontTilt() > initialTilt + 2) {
                telemetry.addData("tilt", frontTilt());
                encoderDrive(APPROACH_SPEED * .6, -.5, -.5, 1);
                if (color == "blue")
                    spinLeft();
                else if (color == "red")
                    spinRight();
            }

            idle();
        }
        // Stop all motion;
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  sleep(250);   // optional pause after each move
    }

    public void encoderDriveSpinup(double speed,
                                   double leftInches, double rightInches,
                                   double timeoutS) throws InterruptedException {

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ElapsedTime runtime = new ElapsedTime();
        int newLeftTarget;
        int newRightTarget;

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        newLeftTarget = leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newRightTarget = rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
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

            leftShooterPowerMgr.regulatePower();
            rightShooterPowerMgr.regulatePower();

            idle();
        }
        // Stop all motion;
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //  sleep(250);   // optional pause after each move
    }

    public void spinup(double seconds) {
        ElapsedTime shootTime = new ElapsedTime();
        while (opMode.opModeIsActive() && shootTime.seconds() < seconds) {
            leftShooterPowerMgr.regulatePower();
            rightShooterPowerMgr.regulatePower();
        }
    }

    public void shoot() {
        leftServoPos = LEFT_IN_VAL;
        rightServoPos = RIGHT_IN_VAL;

        ElapsedTime shootTime = new ElapsedTime();
        scooper.setPower(1);
        while (opMode.opModeIsActive() && shootTime.seconds() < .5) {
            leftShooterPowerMgr.regulatePower();
            rightShooterPowerMgr.regulatePower();
        }
        scooper.setPower(0);

        leftArm.setPosition(leftServoPos);
        rightArm.setPosition(rightServoPos);

        shootTime.reset();
        while (opMode.opModeIsActive() && shootTime.seconds() < .75) {
            leftShooterPowerMgr.regulatePower();
            rightShooterPowerMgr.regulatePower();
        }
        shooter1.setPower(0);
        shooter2.setPower(0);

        leftServoPos = LEFT_OUT_VAL;//if we are running the chain up, then extend the servos so they don't break
        rightServoPos = RIGHT_OUT_VAL;//if we are running the chain up, then extend the servos so they don't break
        leftArm.setPosition(leftServoPos);
        rightArm.setPosition(rightServoPos);
    }
}