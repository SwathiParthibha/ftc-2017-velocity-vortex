package org.firstinspires.ftc.teamcode.Pranav;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a our robot.
 * Our files for usage examples.
 */

/*

This is our Class where all our functions are included that are used in the entire project
Please do not edit any of the functions as it will affect other programs in the project

If you have accomplished something please tell us so we can implement into the class.
*/
public class MecanumHardware extends LinearOpMode
{
    /* Public OpMode members. */

    //Currently there are two motors defined. As the season progresses we may add additional motors
    public DcMotor frontRight = null;
    public DcMotor backRight = null;
    public DcMotor frontLeft = null;
    public DcMotor backLeft = null;

    //Where all Sensors are defined
    public ModernRoboticsI2cGyro sensorGyro = null;
    public LightSensor sensorLine = null;
    public UltrasonicSensor sensorUltra = null;
    public ModernRoboticsI2cRangeSensor sensorRange = null;
    public ColorSensor sensorColorRight = null;
    public ColorSensor sensorColorLeft = null;
    //public ModernRoboticsI2cRangeSensor rangeSensor = null;


    //1000 Milliseconds
    public int SECOND = 1000;

    //Motor Variables
    public int ROTATION = 1220; // # of ticks
    public int MOTOR_POWER = 1;

    //Line Sensor Thresholds
    double WHITE_LINE = 0.56;
    double BLACK_MAT = 0.36;

    //Z-Axis of the Modern Robotics Gyro Sensor
    int heading = 0;

    /* local OpMode members. */
    HardwareMap hwMap;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor *///Empty Constructor
    public MecanumHardware()
    {

    }

    /***
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs)
    {
        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
        {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    public void defineMotors()
    {
        frontRight = hwMap.dcMotor.get("motor_2");
        backRight = hwMap.dcMotor.get("motor_4");
        frontLeft = hwMap.dcMotor.get("motor_3");
        backLeft = hwMap.dcMotor.get("motor_1");
    }

    public void defineSenors()
    {
        sensorGyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
        sensorLine = hwMap.lightSensor.get("line");
        sensorUltra = hwMap.ultrasonicSensor.get("ultra");
        sensorRange = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        sensorColorLeft = hwMap.get(ModernRoboticsI2cColorSensor.class, "colorLeft");
        sensorColorRight = hwMap.get(ModernRoboticsI2cColorSensor.class, "colorRight");
    }

    //Configure the Direction of the Motors
    public void setDirectionMotors()
    {

        frontRight.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backRight.setDirection(DcMotor.Direction.REVERSE); // Set to FORWARD if using AndyMark motors
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
    }

    public void initializeSensors()
    {
        //Calibrate the Modern Robotics Gyro Sensor
        sensorGyro.calibrate();

        //Turn on the LED of the Lego Line Sensor
        sensorLine.enableLed(true);

        //Set the i2c address of one of the color sensors.
        sensorColorLeft.setI2cAddress(new I2cAddr(0x4c));

        //Turn off the LED on the Modern Robotics Color Sensor
        sensorColorLeft.enableLed(false);
        sensorColorRight.enableLed(false);
    }

    public void setMotorPower(double power)
    {
        frontRight.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        backLeft.setPower(power);
    }
    //This function sets the motors to 0 stopping the Robot
    public void stopRobot()
    {
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
    }

    public void stopAndResetEncoder()
    {
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runToPosition()
    {
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runUsingEncoder()
    {
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runWithoutEncoder()
    {
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //A basic go straight function that uses encoders to track its distance
    public void drive(int distance, double speed) throws InterruptedException
    {
        telemetry.addData("Starting to Drive", frontRight.getCurrentPosition() / ROTATION);
        telemetry.update();

        runUsingEncoder();

        stopAndResetEncoder();

        setMotorPower(speed);

        frontRight.setTargetPosition(distance);
        backRight.setTargetPosition(distance);
        frontLeft.setTargetPosition(distance);
        backLeft.setTargetPosition(distance);

        runToPosition();

        while (frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy() && opModeIsActive())
        {

        }

        stopRobot();

        runUsingEncoder();

        telemetry.addData("Finished Driving", frontRight.getCurrentPosition() / ROTATION);
        telemetry.update();
    }

    public void drivePID(int distance, double speed, double angle)
    {
        double currentHeading, headingError;
        double DRIVE_KP = 0.05; // This value relates the degree of error to percentage of motor speed
        double correction, steeringSpeedRight, steeringSpeedLeft;

        telemetry.addData("Starting to Drive Straight", frontRight.getCurrentPosition() / ROTATION);

        runUsingEncoder();

        stopAndResetEncoder();

        frontRight.setTargetPosition(distance);
        backRight.setTargetPosition(distance);
        frontLeft.setTargetPosition(distance);
        backLeft.setTargetPosition(distance);

        setMotorPower(speed);

        runToPosition();

        while (frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy() && opModeIsActive())
        {
            currentHeading = sensorGyro.getHeading();
            headingError = currentHeading - angle;
            correction = headingError * DRIVE_KP;

            // We will correct the direction by changing the motor speeds while the robot drives
            steeringSpeedLeft = (speed * MOTOR_POWER) - correction;
            steeringSpeedRight = (speed * MOTOR_POWER) + correction;

            //Making sure that the Motors are not commanded to go greater than the maximum speed
            steeringSpeedLeft = Range.clip(steeringSpeedLeft,-1,1);
            steeringSpeedRight = Range.clip(steeringSpeedRight,-1,1);

            runUsingEncoder();

            frontRight.setPower(steeringSpeedRight);
            backRight.setPower(steeringSpeedRight);
            frontLeft.setPower(steeringSpeedLeft);
            backLeft.setPower(steeringSpeedLeft);

            runToPosition();

            sleep(100);

            if ((frontRight.getCurrentPosition() > distance)) break;
            if ((frontLeft.getCurrentPosition() > distance)) break;
            if ((backRight.getCurrentPosition() > distance)) break;
            if ((backLeft.getCurrentPosition() > distance)) break;

            telemetry.addData("PID Values", null);
            telemetry.addData("Current Heading:", currentHeading);
            telemetry.addData("Heading Error:", headingError);
            telemetry.addData("Correction:", correction);

            telemetry.addData("Motor Power Values", null);
            telemetry.addData("Steering Speed Right:", steeringSpeedRight);
            telemetry.addData("Steering Speed Left:", steeringSpeedLeft);
            telemetry.addData("Front Right Power:", frontRight.getPower());
            telemetry.addData("Front Left Power:", frontLeft.getPower());
            telemetry.addData("Back Right Power:", backRight.getPower());
            telemetry.update();
        }

        stopRobot();

        runUsingEncoder();

        telemetry.addData("Finished Driving", frontRight.getCurrentPosition() / ROTATION);

    }

    public void pushButton()
    {
        boolean atBeacon = false;

        while(!atBeacon)
        {
            telemetry.addData("Right Color Sensor Values", null);
            telemetry.addData("Right Red:", sensorColorRight.red());
            telemetry.addData("Right Blue:", sensorColorRight.blue());
            telemetry.addData("Right Green:", sensorColorRight.green());
            telemetry.addData("Left Color Sensor Values", null);
            telemetry.addData("Left Red:", sensorColorLeft.red());
            telemetry.addData("Left Blue:", sensorColorLeft.blue());
            telemetry.addData("Left Green:", sensorColorLeft.green());
            telemetry.update();
        }

    }

    public void lineFollower(double speed)
    {
        runWithoutEncoder();

        double currentLight, lineError, lineCorrection;
        double LINE_FOLLOW_KP = 0.1;
        double steeringSpeedLeft, steeringSpeedRight;

        while(opModeIsActive())
        {
            currentLight = sensorLine.getLightDetected();
            lineError = currentLight - WHITE_LINE;
            lineCorrection = LINE_FOLLOW_KP * lineError;

            steeringSpeedLeft = -(speed - lineCorrection);
            steeringSpeedRight = -(speed + lineCorrection);

            steeringSpeedLeft = Range.clip(steeringSpeedLeft, -1, 1);
            steeringSpeedRight= Range.clip(steeringSpeedRight, -1, 1);

            frontRight.setPower(steeringSpeedRight);
            backRight.setPower(steeringSpeedRight);
            frontLeft.setPower(steeringSpeedLeft);
            backLeft.setPower(steeringSpeedLeft);

            sleep(1);

            telemetry.addData("Line Sensor Values", null);
            telemetry.addData("Current Light:", currentLight);
            telemetry.addData("Line Error:", lineError);
            telemetry.addData("Line Correction:", lineCorrection);

            telemetry.addData("Motor Powers", null);
            telemetry.addData("Front Right:",frontRight.getPower());
            telemetry.addData("Back Right:", backRight.getPower());
            telemetry.addData("Front Left:", frontLeft.getPower());
            telemetry.addData("Back Left;", backLeft.getPower());
            telemetry.update();
        }
    }

    //A basic Turn function that uses the Modern Robotics Gyro Sensor to calculate the angle
    public void turnGyro(String direction, int angle, double speed) throws InterruptedException
    {
        int motorDirectionChange = 0;

        runWithoutEncoder();

        if (direction.equals("left"))
        {
            motorDirectionChange = 1;
        }
        else
        if (direction.equals("right"))
        {
            motorDirectionChange = -1;
        }

        while ((heading > angle + 5 || heading < angle - 2 && opModeIsActive()))
        {
            frontRight.setPower(MOTOR_POWER * speed * motorDirectionChange);
            backRight.setPower(MOTOR_POWER * speed * motorDirectionChange);

            frontLeft.setPower(-MOTOR_POWER * speed * motorDirectionChange);
            backLeft.setPower(-MOTOR_POWER * speed * motorDirectionChange);

            heading = sensorGyro.getHeading();

            telemetry.addData("We Are Turning", heading);
            telemetry.addData("Gyro Value", sensorGyro.getHeading());
            telemetry.update();
        }

        stopRobot();

        telemetry.addData("We Are Done Turning", heading);
    }

    public void init(HardwareMap ahwMap)
    {
        //Save reference to Hardware Map
        hwMap = ahwMap;

        defineMotors();

        defineSenors();

        setDirectionMotors();

        stopRobot();

        runUsingEncoder();

        initializeSensors();
    }
    @Override
    public void runOpMode() throws InterruptedException
    {

    }
}

