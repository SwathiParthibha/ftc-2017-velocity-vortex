package org.firstinspires.ftc.teamcode.Pranav;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Thread.sleep;


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
class MecanumHardware
{
    /* Public OpMode members. */

    //Currently there are two motors defined. As the season progresses we may add additional motors
    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor backLeft;

    //Where all Sensors are defined
    public ModernRoboticsI2cGyro sensorGyro;
    public LightSensor sensorLine;
    public UltrasonicSensor sensorUltra;
    public ModernRoboticsI2cRangeSensor sensorRange;
    public ModernRoboticsAnalogOpticalDistanceSensor sensorODS;
    public ColorSensor sensorColorRight;
    public ColorSensor sensorColorLeft;
    public BNO055IMU imu;

    public OpMode opMode;

    //1000 Milliseconds
    public int SECOND = 1000;

    //Endoder Motor Variables
    public int ROTATION = 1220; // # of ticks
    public int MOTOR_POWER = 1;

    //Z-Axis of the Modern Robotics Gyro Sensor
    public int heading = 0;

    //Light Sensor Threshold Value
    //Make sure to add one.
    public double LINE_THRESHOLD_VALUE = 0.28;


    //Heading for the IMU;
    double angles;


    /* local OpMode members. */
    HardwareMap hardwareMap;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor *///Empty Constructor
    public MecanumHardware(OpMode opMode)
    {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
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
                sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    //This function defines the motors so that the robot controller looks for
    public void defineMotors()
    {
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
    }

    //This function defines the sensors so that the robot controller looks for
    public void defineSensors()
    {
        sensorGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        //sensorLine = hardwareMap.lightSensor.get("line");
        //sensorUltra = hardwareMap.ultrasonicSensor.get("ultra");
        //sensorRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        sensorColorLeft = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "colorLeft");
        sensorColorRight = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "colorRight");
        sensorODS = hardwareMap.get(ModernRoboticsAnalogOpticalDistanceSensor.class, "sensorODS");
    }

    //Configure the Direction of the Motors
    public void setDirectionMotors()
    {
        frontRight.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backRight.setDirection(DcMotor.Direction.REVERSE); // Set to FORWARD if using AndyMark motors
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
    }

    //Initializes the sensors with certain configuartions
    public void initializeSensors()
    {
        //Calibrate the Modern Robotics Gyro Sensor
        //sensorGyro.calibrate();

        /*
        //Turn on the LED of the Lego Line Sensor
        sensorLine.enableLed(true);

        */

        //Set the i2c address of one of the color sensors.
        I2cAddr i2cAddr = I2cAddr.create8bit(0x4c);
        sensorColorLeft.setI2cAddress(i2cAddr);

        //Turn off the LED on the Modern Robotics Color Sensor
        sensorColorLeft.enableLed(true);
        sensorColorRight.enableLed(true);

        sensorColorLeft.enableLed(false);
        sensorColorRight.enableLed(false);

        sensorODS.enableLed(true);

        //digitalChannelController.setDigitalChannelMode(7, DigitalChannelController.Mode.INPUT);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    //Set the all the motors to a set power
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

    //Sets all the motors to the STOP_AND_RESET_ENCODER Mode
        public void stopAndResetEncoder()
    {
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Sets all the motors to the RUN_TO_POSITION
    public void runToPosition()
    {
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //Sets all the motors to the RUN_USING_ENCODER
    public void runUsingEncoder()
    {
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Sets all the motors to the RUN_WITHOUT_ENCODER
    public void runWithoutEncoder()
    {
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double IMUHeading()
    {
        return ((imu.getAngularOrientation().firstAngle) * -1);
    }

    //A basic go straight function that uses encoders to track its distance
    public void drive(int distance, double speed)
    {
        opMode.telemetry.addData("Starting to Drive", frontRight.getCurrentPosition() / ROTATION);
        opMode.telemetry.update();

        runUsingEncoder();

        stopAndResetEncoder();

        setMotorPower(speed);

        frontRight.setTargetPosition(distance);
        backRight.setTargetPosition(distance);
        frontLeft.setTargetPosition(distance);
        backLeft.setTargetPosition(distance);

        runToPosition();

        while (frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy())
        {

        }

        stopRobot();

        runUsingEncoder();

        opMode.telemetry.addData("Finished Driving", frontRight.getCurrentPosition() / ROTATION);
        opMode.telemetry.update();
    }

    //Runs an individual motor to a set distance using encoders
    public void driveIndividualMotor(String motor, int distance, double speed)
    {
        if (motor == "frontRight")
            {
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontRight.setPower(speed);

            frontRight.setTargetPosition(distance);

            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (frontRight.isBusy())
            {

            }

            frontRight.setPower(0);

            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (motor == "frontLeft")
        {
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeft.setPower(speed);

            frontLeft.setTargetPosition(distance);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (frontLeft.isBusy())
            {

            }

            frontLeft.setPower(0);

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (motor == "backRight")
        {
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            backRight.setPower(speed);

            backRight.setTargetPosition(distance);

            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (backRight.isBusy())
            {

            }

            backRight.setPower(0);

            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (motor == "backLeft")
        {
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            backLeft.setPower(speed);

            backLeft.setTargetPosition(distance);

            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (backLeft.isBusy())
            {

            }

            backLeft.setPower(0);

            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

    public void driveDiagonalIMU(String direction, int distance, double speed, int targetAngle) throws InterruptedException
    {
        double currentHeading, headingError;
        double DRIVE_KP =0.05; //This value relates the degree of error to percentage of motor speed
        double correction, steeringSpeedRight, steeringSpeedLeft;

        if (direction == "NW")
        {
            runUsingEncoder();

            stopAndResetEncoder();

            frontRight.setPower(speed);
            backLeft.setPower(speed);

            frontRight.setTargetPosition(distance);
            backLeft.setTargetPosition(distance);

            runToPosition();

            while (frontRight.isBusy() && backLeft.isBusy())
            {
                if (sensorODS.getLightDetected() > LINE_THRESHOLD_VALUE) break;

                currentHeading = IMUHeading();
                headingError = currentHeading - targetAngle;
                correction = headingError * DRIVE_KP;

                // We will correct the direction by changing the motor speeds while the robot drives
                steeringSpeedLeft = (speed * MOTOR_POWER) - correction;
                steeringSpeedRight = (speed * MOTOR_POWER) + correction;

                //Making sure that the Motors are not commanded to go greater than the maximum speed
                steeringSpeedLeft = Range.clip(steeringSpeedLeft,-1,1);
                steeringSpeedRight = Range.clip(steeringSpeedRight,-1,1);

                runUsingEncoder();

                frontRight.setPower(steeringSpeedRight);
                backLeft.setPower(steeringSpeedLeft);

                runToPosition();

                try {
                    sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }



                if ((frontRight.getCurrentPosition() > distance)) break;
                if ((frontLeft.getCurrentPosition() > distance)) break;
                if ((backRight.getCurrentPosition() > distance)) break;
                if ((backLeft.getCurrentPosition() > distance)) break;

                opMode.telemetry.addData("PID Values", null);
                opMode.telemetry.addData("Current Heading:", currentHeading);
                opMode.telemetry.addData("Heading Error:", headingError);
                opMode.telemetry.addData("Correction:", correction);

                opMode.telemetry.addData("ShooterMotor Power Values", null);
                opMode.telemetry.addData("Steering Speed Right:", steeringSpeedRight);
                opMode.telemetry.addData("Steering Speed Left:", steeringSpeedLeft);
                opMode.telemetry.addData("Front Right Power:", frontRight.getPower());
                opMode.telemetry.addData("Front Left Power:", frontLeft.getPower());
                opMode.telemetry.addData("Back Right Power:", backRight.getPower());
                opMode.telemetry.update();




            }

            stopRobot();

            runUsingEncoder();

            opMode.telemetry.addData("Finished Driving", frontRight.getCurrentPosition() / ROTATION);
            opMode.telemetry.update();
        }

        if(direction == "NE")
        {
            runUsingEncoder();
            stopAndResetEncoder();

            frontLeft.setPower(speed);
            backRight.setPower(speed);

            backRight.setTargetPosition(distance);
            frontLeft.setTargetPosition(distance);

            runToPosition();

            while (backRight.isBusy() && frontLeft.isBusy())
            {
                currentHeading = IMUHeading();
                headingError = currentHeading - targetAngle;
                correction = headingError * DRIVE_KP;

                // We will correct the direction by changing the motor speeds while the robot drives
                steeringSpeedLeft = (speed * MOTOR_POWER) - correction;
                steeringSpeedRight = (speed * MOTOR_POWER) + correction;

                //Making sure that the Motors are not commanded to go greater than the maximum speed
                steeringSpeedLeft = Range.clip(steeringSpeedLeft,-1,1);
                steeringSpeedRight = Range.clip(steeringSpeedRight,-1,1);

                runUsingEncoder();

                backRight.setPower(steeringSpeedRight);
                frontLeft.setPower(steeringSpeedLeft);

                runToPosition();

                try {
                    sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                if ((frontRight.getCurrentPosition() > distance)) break;
                if ((frontLeft.getCurrentPosition() > distance)) break;
                if ((backRight.getCurrentPosition() > distance)) break;
                if ((backLeft.getCurrentPosition() > distance)) break;

                opMode.telemetry.addData("PID Values", null);
                opMode.telemetry.addData("Current Heading:", currentHeading);
                opMode.telemetry.addData("Heading Error:", headingError);
                opMode.telemetry.addData("Correction:", correction);

                opMode.telemetry.addData("ShooterMotor Power Values", null);
                opMode.telemetry.addData("Steering Speed Right:", steeringSpeedRight);
                opMode.telemetry.addData("Steering Speed Left:", steeringSpeedLeft);
                opMode.telemetry.addData("Front Right Power:", frontRight.getPower());
                opMode.telemetry.addData("Front Left Power:", frontLeft.getPower());
                opMode.telemetry.addData("Back Right Power:", backRight.getPower());
                opMode.telemetry.update();

                if (sensorODS.getLightDetected() > LINE_THRESHOLD_VALUE) break;

            }

            stopRobot();

            runUsingEncoder();

            opMode.telemetry.addData("Finished Driving", frontRight.getCurrentPosition() / ROTATION);
            opMode.telemetry.update();
        }

        if (direction == "SW")
        {
            runUsingEncoder();

            stopAndResetEncoder();

            frontRight.setPower(speed);
            backLeft.setPower(speed);

            frontRight.setTargetPosition(-distance);
            backLeft.setTargetPosition(-distance);

            runToPosition();

            while (frontRight.isBusy() && backLeft.isBusy())
            {
                currentHeading = sensorGyro.getHeading();
                headingError = currentHeading - targetAngle;
                correction = headingError * DRIVE_KP;

                // We will correct the direction by changing the motor speeds while the robot drives
                steeringSpeedLeft = (speed * MOTOR_POWER) - correction;
                steeringSpeedRight = (speed * MOTOR_POWER) + correction;

                //Making sure that the Motors are not commanded to go greater than the maximum speed
                steeringSpeedLeft = Range.clip(steeringSpeedLeft,-1,1);
                steeringSpeedRight = Range.clip(steeringSpeedRight,-1,1);

                runUsingEncoder();

                frontRight.setPower(steeringSpeedRight);
                backLeft.setPower(steeringSpeedLeft);

                runToPosition();

                try {
                    sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                if ((frontRight.getCurrentPosition() > distance)) break;
                if ((frontLeft.getCurrentPosition() > distance)) break;
                if ((backRight.getCurrentPosition() > distance)) break;
                if ((backLeft.getCurrentPosition() > distance)) break;

                opMode.telemetry.addData("PID Values", null);
                opMode.telemetry.addData("Current Heading:", currentHeading);
                opMode.telemetry.addData("Heading Error:", headingError);
                opMode.telemetry.addData("Correction:", correction);

                opMode.telemetry.addData("ShooterMotor Power Values", null);
                opMode.telemetry.addData("Steering Speed Right:", steeringSpeedRight);
                opMode.telemetry.addData("Steering Speed Left:", steeringSpeedLeft);
                opMode.telemetry.addData("Front Right Power:", frontRight.getPower());
                opMode.telemetry.addData("Front Left Power:", frontLeft.getPower());
                opMode.telemetry.addData("Back Right Power:", backRight.getPower());
                opMode.telemetry.update();



                if (sensorODS.getLightDetected() > LINE_THRESHOLD_VALUE) break;

            }

            stopRobot();

            runUsingEncoder();

            opMode.telemetry.addData("Finished Driving", frontRight.getCurrentPosition() / ROTATION);
            opMode.telemetry.update();
        }

        if (direction == "SE")
        {
            runUsingEncoder();
            stopAndResetEncoder();

            frontLeft.setPower(speed);
            backRight.setPower(speed);

            backRight.setTargetPosition(-distance);
            frontLeft.setTargetPosition(-distance);

            runToPosition();

            while (backRight.isBusy() && frontLeft.isBusy())
            {
                currentHeading = IMUHeading();
                headingError = currentHeading - targetAngle;
                correction = headingError * DRIVE_KP;

                // We will correct the direction by changing the motor speeds while the robot drives
                steeringSpeedLeft = (speed * MOTOR_POWER) - correction;
                steeringSpeedRight = (speed * MOTOR_POWER) + correction;

                //Making sure that the Motors are not commanded to go greater than the maximum speed
                steeringSpeedLeft = Range.clip(steeringSpeedLeft,-1,1);
                steeringSpeedRight = Range.clip(steeringSpeedRight,-1,1);

                runUsingEncoder();

                backRight.setPower(steeringSpeedRight);
                frontLeft.setPower(steeringSpeedLeft);

                runToPosition();

                try {
                    sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                if ((frontRight.getCurrentPosition() > distance)) break;
                if ((frontLeft.getCurrentPosition() > distance)) break;
                if ((backRight.getCurrentPosition() > distance)) break;
                if ((backLeft.getCurrentPosition() > distance)) break;

                opMode.telemetry.addData("PID Values", null);
                opMode.telemetry.addData("Current Heading:", currentHeading);
                opMode.telemetry.addData("Heading Error:", headingError);
                opMode.telemetry.addData("Correction:", correction);

                opMode.telemetry.addData("ShooterMotor Power Values", null);
                opMode.telemetry.addData("Steering Speed Right:", steeringSpeedRight);
                opMode.telemetry.addData("Steering Speed Left:", steeringSpeedLeft);
                opMode.telemetry.addData("Front Right Power:", frontRight.getPower());
                opMode.telemetry.addData("Front Left Power:", frontLeft.getPower());
                opMode.telemetry.addData("Back Right Power:", backRight.getPower());
                opMode.telemetry.update();

                if (sensorODS.getLightDetected() > LINE_THRESHOLD_VALUE) break;            }

            stopRobot();

            runUsingEncoder();

            opMode.telemetry.addData("Finished Driving", frontRight.getCurrentPosition() / ROTATION);
            opMode.telemetry.update();
        }
    }

    public void driveDiagonalGyro(String direction, int distance, double speed, int angle) throws InterruptedException
    {
        double currentHeading, headingError;
        double DRIVE_KP =0.05; //This value relates the degree of error to percentage of motor speed
        double correction, steeringSpeedRight, steeringSpeedLeft;

        if (direction == "NW")
        {
            runUsingEncoder();

            stopAndResetEncoder();

            frontRight.setPower(speed);
            backLeft.setPower(speed);

            frontRight.setTargetPosition(distance);
            backLeft.setTargetPosition(distance);

            runToPosition();

            while (frontRight.isBusy() && backLeft.isBusy())
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
                backLeft.setPower(steeringSpeedLeft);

                runToPosition();

                try {
                    sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                if ((frontRight.getCurrentPosition() > distance)) break;
                if ((frontLeft.getCurrentPosition() > distance)) break;
                if ((backRight.getCurrentPosition() > distance)) break;
                if ((backLeft.getCurrentPosition() > distance)) break;

                opMode.telemetry.addData("PID Values", null);
                opMode.telemetry.addData("Current Heading:", currentHeading);
                opMode.telemetry.addData("Heading Error:", headingError);
                opMode.telemetry.addData("Correction:", correction);

                opMode.telemetry.addData("ShooterMotor Power Values", null);
                opMode.telemetry.addData("Steering Speed Right:", steeringSpeedRight);
                opMode.telemetry.addData("Steering Speed Left:", steeringSpeedLeft);
                opMode.telemetry.addData("Front Right Power:", frontRight.getPower());
                opMode.telemetry.addData("Front Left Power:", frontLeft.getPower());
                opMode.telemetry.addData("Back Right Power:", backRight.getPower());
                opMode.telemetry.update();



                if (sensorODS.getLightDetected() > LINE_THRESHOLD_VALUE) break;
            }

            stopRobot();

            runUsingEncoder();

            opMode.telemetry.addData("Finished Driving", frontRight.getCurrentPosition() / ROTATION);
            opMode.telemetry.update();
        }

        if(direction == "NE")
        {
            runUsingEncoder();
            stopAndResetEncoder();

            frontLeft.setPower(speed);
            backRight.setPower(speed);

            backRight.setTargetPosition(distance);
            frontLeft.setTargetPosition(distance);

            runToPosition();

            while (backRight.isBusy() && frontLeft.isBusy())
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

                backRight.setPower(steeringSpeedRight);
                frontLeft.setPower(steeringSpeedLeft);

                runToPosition();

                try {
                    sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                if ((frontRight.getCurrentPosition() > distance)) break;
                if ((frontLeft.getCurrentPosition() > distance)) break;
                if ((backRight.getCurrentPosition() > distance)) break;
                if ((backLeft.getCurrentPosition() > distance)) break;

                opMode.telemetry.addData("PID Values", null);
                opMode.telemetry.addData("Current Heading:", currentHeading);
                opMode.telemetry.addData("Heading Error:", headingError);
                opMode.telemetry.addData("Correction:", correction);

                opMode.telemetry.addData("ShooterMotor Power Values", null);
                opMode.telemetry.addData("Steering Speed Right:", steeringSpeedRight);
                opMode.telemetry.addData("Steering Speed Left:", steeringSpeedLeft);
                opMode.telemetry.addData("Front Right Power:", frontRight.getPower());
                opMode.telemetry.addData("Front Left Power:", frontLeft.getPower());
                opMode.telemetry.addData("Back Right Power:", backRight.getPower());
                opMode.telemetry.update();

                if (sensorODS.getLightDetected() > LINE_THRESHOLD_VALUE) break;

            }

            stopRobot();

            runUsingEncoder();

            opMode.telemetry.addData("Finished Driving", frontRight.getCurrentPosition() / ROTATION);
            opMode.telemetry.update();
        }

        if (direction == "SW")
        {
            runUsingEncoder();

            stopAndResetEncoder();

            frontRight.setPower(speed);
            backLeft.setPower(speed);

            frontRight.setTargetPosition(-distance);
            backLeft.setTargetPosition(-distance);

            runToPosition();

            while (frontRight.isBusy() && backLeft.isBusy())
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
                backLeft.setPower(steeringSpeedLeft);

                runToPosition();

                try {
                    sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                if ((frontRight.getCurrentPosition() > distance)) break;
                if ((frontLeft.getCurrentPosition() > distance)) break;
                if ((backRight.getCurrentPosition() > distance)) break;
                if ((backLeft.getCurrentPosition() > distance)) break;

                opMode.telemetry.addData("PID Values", null);
                opMode.telemetry.addData("Current Heading:", currentHeading);
                opMode.telemetry.addData("Heading Error:", headingError);
                opMode.telemetry.addData("Correction:", correction);

                opMode.telemetry.addData("ShooterMotor Power Values", null);
                opMode.telemetry.addData("Steering Speed Right:", steeringSpeedRight);
                opMode.telemetry.addData("Steering Speed Left:", steeringSpeedLeft);
                opMode.telemetry.addData("Front Right Power:", frontRight.getPower());
                opMode.telemetry.addData("Front Left Power:", frontLeft.getPower());
                opMode.telemetry.addData("Back Right Power:", backRight.getPower());
                opMode.telemetry.update();



                if (sensorODS.getLightDetected() > LINE_THRESHOLD_VALUE) break;

            }

            stopRobot();

            runUsingEncoder();

            opMode.telemetry.addData("Finished Driving", frontRight.getCurrentPosition() / ROTATION);
            opMode.telemetry.update();
        }

        if (direction == "SE")
        {
            runUsingEncoder();
            stopAndResetEncoder();

            frontLeft.setPower(speed);
            backRight.setPower(speed);

            backRight.setTargetPosition(-distance);
            frontLeft.setTargetPosition(-distance);

            runToPosition();

            while (backRight.isBusy() && frontLeft.isBusy())
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

                backRight.setPower(steeringSpeedRight);
                frontLeft.setPower(steeringSpeedLeft);

                runToPosition();

                try {
                    sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                if ((frontRight.getCurrentPosition() > distance)) break;
                if ((frontLeft.getCurrentPosition() > distance)) break;
                if ((backRight.getCurrentPosition() > distance)) break;
                if ((backLeft.getCurrentPosition() > distance)) break;

                opMode.telemetry.addData("PID Values", null);
                opMode.telemetry.addData("Current Heading:", currentHeading);
                opMode.telemetry.addData("Heading Error:", headingError);
                opMode.telemetry.addData("Correction:", correction);

                opMode.telemetry.addData("ShooterMotor Power Values", null);
                opMode.telemetry.addData("Steering Speed Right:", steeringSpeedRight);
                opMode.telemetry.addData("Steering Speed Left:", steeringSpeedLeft);
                opMode.telemetry.addData("Front Right Power:", frontRight.getPower());
                opMode.telemetry.addData("Front Left Power:", frontLeft.getPower());
                opMode.telemetry.addData("Back Right Power:", backRight.getPower());
                opMode.telemetry.update();

                if (sensorODS.getLightDetected() > LINE_THRESHOLD_VALUE) break;            }

            stopRobot();

            runUsingEncoder();

            opMode.telemetry.addData("Finished Driving", frontRight.getCurrentPosition() / ROTATION);
            opMode.telemetry.update();
        }
    }

    public void driveSideways(String direction, int distance, double speed)
    {
        if (direction == "right")
        {
            runUsingEncoder();

            stopAndResetEncoder();

            setMotorPower(speed);

            frontLeft.setTargetPosition(distance);
            frontRight.setTargetPosition(-distance);
            backLeft.setTargetPosition(-distance);
            backRight.setTargetPosition(distance);

            runToPosition();

            while (frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy())
            {

            }

            stopRobot();

            runUsingEncoder();

            opMode.telemetry.addData("Finished Driving", frontRight.getCurrentPosition() / ROTATION);
            opMode.telemetry.update();
        }

        if (direction == "left")
        {
            runUsingEncoder();

            stopAndResetEncoder();

            setMotorPower(speed);

            frontLeft.setTargetPosition(-distance);
            frontRight.setTargetPosition(distance);
            backLeft.setTargetPosition(distance);
            backRight.setTargetPosition(-distance);

            runToPosition();

            while (frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy())
            {

            }

            stopRobot();

            runUsingEncoder();

            opMode.telemetry.addData("Finished Driving", frontRight.getCurrentPosition() / ROTATION);
            opMode.telemetry.update();
        }
    }


    //Reverses the configuration of the motor directions
    public void reverseDirectionMotors()
    {

        frontRight.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        backRight.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    public void mecanumDriveCartesian(double xPosition, double yPosition, double rotation, double gyroAngle)
    {
        double GYRO_ASSIST_KP = 0.01;
        double GYRO_RATE_SCALE = 0.01;

        double speedFrontLeftMotor;
        double speedFrontRightMotor;
        double speedBackLeftMotor;
        double speedBackRightMotor;

        xPosition = Range.clip(xPosition,-1,1);
        yPosition = Range.clip(yPosition,-1,1);
        rotation = Range.clip(rotation,-1,1);

        double cosA = Math.cos(Math.toRadians(gyroAngle));
        double sinA = Math.sin(Math.toRadians(gyroAngle));

        xPosition = (xPosition * cosA) - (yPosition * sinA);
        yPosition = (xPosition * sinA) + (yPosition * cosA);

        rotation += Range.clip(GYRO_ASSIST_KP * (rotation - GYRO_RATE_SCALE * heading), -1, 1);

        speedFrontLeftMotor = xPosition + yPosition + rotation;
        speedFrontRightMotor = (-xPosition) + yPosition - rotation;
        speedBackLeftMotor = (-xPosition) + yPosition + rotation;
        speedBackRightMotor = xPosition + yPosition - rotation;

        speedFrontLeftMotor = Range.clip(speedFrontLeftMotor,-1,1);
        speedFrontRightMotor = Range.clip(speedFrontRightMotor,-1,1);
        speedBackLeftMotor = Range.clip(speedBackLeftMotor,-1,1);
        speedBackRightMotor = Range.clip(speedBackRightMotor,-1,1);

        frontLeft.setPower(speedFrontLeftMotor);
        frontRight.setPower(speedFrontRightMotor);
        backLeft.setPower(speedBackLeftMotor);
        backRight.setPower(speedBackRightMotor);


        opMode.telemetry.addData("Motor Powers ", null);
        opMode.telemetry.addData("Front Left:", frontLeft.getPower());
        opMode.telemetry.addData("Front Right:", frontRight.getPower());
        opMode.telemetry.addData("Back Left:", backLeft.getPower());
        opMode.telemetry.addData("back Right:", backRight.getPower());

        opMode.telemetry.addData("Gyro Angle:", heading);

    }

    //A go straight program that utilizes PID using the gyro sensor to stay accurate
    public void drivePID(int distance, double speed, double angle)
    {
        double currentHeading, headingError;
        double DRIVE_KP =0.05; //This value relates the degree of error to percentage of motor speed
        double correction, steeringSpeedRight, steeringSpeedLeft;

        opMode.telemetry.addData("Starting to Drive Straight", frontRight.getCurrentPosition() / ROTATION);

        runUsingEncoder();

        stopAndResetEncoder();

        frontRight.setTargetPosition(distance);
        backRight.setTargetPosition(distance);
        frontLeft.setTargetPosition(distance);
        backLeft.setTargetPosition(distance);

        setMotorPower(speed);

        runToPosition();

        while (frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy() )
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

            try {
                sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            if ((frontRight.getCurrentPosition() > distance)) break;
            if ((frontLeft.getCurrentPosition() > distance)) break;
            if ((backRight.getCurrentPosition() > distance)) break;
            if ((backLeft.getCurrentPosition() > distance)) break;

            opMode.telemetry.addData("PID Values", null);
            opMode.telemetry.addData("Current Heading:", currentHeading);
            opMode.telemetry.addData("Heading Error:", headingError);
            opMode.telemetry.addData("Correction:", correction);

            opMode.telemetry.addData("ShooterMotor Power Values", null);
            opMode.telemetry.addData("Steering Speed Right:", steeringSpeedRight);
            opMode.telemetry.addData("Steering Speed Left:", steeringSpeedLeft);
            opMode.telemetry.addData("Front Right Power:", frontRight.getPower());
            opMode.telemetry.addData("Front Left Power:", frontLeft.getPower());
            opMode.telemetry.addData("Back Right Power:", backRight.getPower());
            opMode.telemetry.update();
        }

        stopRobot();

        runUsingEncoder();

        opMode.telemetry.addData("Finished Driving", frontRight.getCurrentPosition() / ROTATION);

    }

    //A go straight program that utilizes PID using the gyro sensor to stay accurate
    public void drivePIDIMU(int distance, double speed, double targetAngle)
    {
        double currentHeading, headingError;
        double DRIVE_KP =0.05; //This value relates the degree of error to percentage of motor speed
        double correction, steeringSpeedRight, steeringSpeedLeft;

        opMode.telemetry.addData("Starting to Drive Straight", frontRight.getCurrentPosition() / ROTATION);

        runUsingEncoder();

        stopAndResetEncoder();

        frontRight.setTargetPosition(distance);
        backRight.setTargetPosition(distance);
        frontLeft.setTargetPosition(distance);
        backLeft.setTargetPosition(distance);

        setMotorPower(speed);

        runToPosition();

        while (frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy() )
        {
            currentHeading = IMUHeading();
            headingError = currentHeading - targetAngle;
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

            try {
                sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            if ((frontRight.getCurrentPosition() > distance)) break;
            if ((frontLeft.getCurrentPosition() > distance)) break;
            if ((backRight.getCurrentPosition() > distance)) break;
            if ((backLeft.getCurrentPosition() > distance)) break;

            opMode.telemetry.addData("PID Values", null);
            opMode.telemetry.addData("Current Heading:", currentHeading);
            opMode.telemetry.addData("Heading Error:", headingError);
            opMode.telemetry.addData("Correction:", correction);

            opMode.telemetry.addData("Shooter Motor Power Values", null);
            opMode.telemetry.addData("Steering Speed Right:", steeringSpeedRight);
            opMode.telemetry.addData("Steering Speed Left:", steeringSpeedLeft);
            opMode.telemetry.addData("Front Right Power:", frontRight.getPower());
            opMode.telemetry.addData("Front Left Power:", frontLeft.getPower());
            opMode.telemetry.addData("Back Right Power:", backRight.getPower());
            opMode.telemetry.update();

            if (sensorODS.getLightDetected() > LINE_THRESHOLD_VALUE) break;
        }

        stopRobot();

        runUsingEncoder();

        opMode.telemetry.addData("Finished Driving", frontRight.getCurrentPosition() / ROTATION);

    }

    //Pushes a button on the beacon to a color
    public void pushButton(String color)
    {
        runWithoutEncoder();

        boolean atBeacon = false;
        boolean startPushButton = false;
        boolean checkBeacons = false;
        int THRESHOLD_COLOR = 3;

        while(!atBeacon)
        {
            if(sensorColorLeft.red() > THRESHOLD_COLOR || sensorColorRight.red() > THRESHOLD_COLOR || sensorColorLeft.blue() > THRESHOLD_COLOR || sensorColorRight.blue() > THRESHOLD_COLOR)
            {
                atBeacon = true;
                startPushButton = true;
            }
        }

        while(startPushButton)
        {
            if(color == "red")
            {
                if (sensorColorLeft.red() >= THRESHOLD_COLOR)
                {
                    drive(-ROTATION / 6, 0.3);
                    driveSideways("left", ROTATION / 12, 0.3);
                    driveSideways("right", ROTATION / 8, 0.3);
                    drive(ROTATION /6, 0.3);
                    break;
                }


                if (sensorColorRight.red() >= THRESHOLD_COLOR)
                {
                    drive(ROTATION / 6, 0.3);
                    driveSideways("left", ROTATION / 8, 0.3);
                    driveSideways("right", ROTATION / 8, 0.3);
                    drive(-ROTATION / 6, 0.3);
                    break;
                }

                startPushButton = false;
                checkBeacons = true;
            }

            if(color == "blue")
            {
                if (sensorColorLeft.blue() >= THRESHOLD_COLOR)
                {
                    drive(-ROTATION / 6, 0.3);
                    driveSideways("left", ROTATION / 12, 0.3);
                    driveSideways("right", ROTATION / 8, 0.3);
                    drive(ROTATION /6, 0.3);
                    break;

                }

                if (sensorColorRight.blue() >= THRESHOLD_COLOR)
                {
                    drive(ROTATION / 6, 0.3);
                    driveSideways("left", ROTATION / 8, 0.3);
                    driveSideways("right", ROTATION / 8, 0.3);
                    drive(-ROTATION / 6, 0.3);
                    break;
                }
            }
        }

        while(checkBeacons)
        {
            if(color == "red")
            {
                if(sensorColorLeft.red() > THRESHOLD_COLOR && sensorColorRight.red() > THRESHOLD_COLOR)
                {
                    checkBeacons = false;
                }
                else
                {
                    checkBeacons = false;
                    startPushButton = true;
                }
            }

            if(color == "blue")
            {
                if(sensorColorLeft.blue() > THRESHOLD_COLOR && sensorColorRight.blue() > THRESHOLD_COLOR)
                {
                    checkBeacons = false;
                }
                else
                {
                    checkBeacons = false;
                    startPushButton = true;
                }
            }
        }

        stopRobot();

        opMode.telemetry.addData("Right Color Sensor Values", null);
        opMode.telemetry.addData("Right Red:", sensorColorRight.red());
        opMode.telemetry.addData("Right Blue:", sensorColorRight.blue());

        opMode.telemetry.addData("Left Color Sensor Values", null);
        opMode.telemetry.addData("Left Red:", sensorColorLeft.red());
        opMode.telemetry.addData("Left Blue:", sensorColorLeft.blue());
        opMode.telemetry.update();
        opMode.telemetry.update();
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

        while ((heading > angle + 5 || heading < angle - 2 ))
        {
            frontRight.setPower(MOTOR_POWER * speed * motorDirectionChange);
            backRight.setPower(MOTOR_POWER * speed * motorDirectionChange);

            frontLeft.setPower(-MOTOR_POWER * speed * motorDirectionChange);
            backLeft.setPower(-MOTOR_POWER * speed * motorDirectionChange);

            heading = sensorGyro.getHeading();

            opMode.telemetry.addData("We Are Turning", heading);
            opMode.telemetry.addData("Gyro Value", sensorGyro.getHeading());
            opMode.telemetry.update();
        }

        stopRobot();

        opMode.telemetry.addData("We Are Done Turning", heading);
    }

    //A basic Turn function that uses the Modern Robotics Gyro Sensor to calculate the angle
    public void turnIMU(String direction, int angle, double speed) throws InterruptedException
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

        while ((IMUHeading() > angle + 5 || IMUHeading() < angle - 2 ))
        {
            frontRight.setPower(MOTOR_POWER * speed * motorDirectionChange);
            backRight.setPower(MOTOR_POWER * speed * motorDirectionChange);

            frontLeft.setPower(-MOTOR_POWER * speed * motorDirectionChange);
            backLeft.setPower(-MOTOR_POWER * speed * motorDirectionChange);

            opMode.telemetry.addData("We Are Turning", null);
            opMode.telemetry.addData("Gyro Value", IMUHeading());
            opMode.telemetry.update();
        }

        stopRobot();

        opMode.telemetry.addData("We Are Done Turning", IMUHeading());
    }


    public boolean init(HardwareMap hardwareMap)
    {
        this.hardwareMap = hardwareMap;


        defineMotors();

        defineSensors();

        setDirectionMotors();

        stopRobot();

        runUsingEncoder();

        initializeSensors();



        return false;
    }

}

