package org.firstinspires.ftc.teamcode.Sam;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a our robot.
 * Our files for usage examples.
 */
//asdjnkasd
/*

This is our Class where all our functions are included that are used in the entire project
Please do not edit any of the functions as it will affect other programs in the project

If you have accomplished something please tell us so we can implement into the class.
*/
    //test
public class RobotHardware
{
    /* Public OpMode members. */

    //Currently there are two motors defined. As the season progresses we may add additional motors
    DcMotor frontRight = null;
    DcMotor backRight = null;
    DcMotor frontLeft = null;
    DcMotor backLeft = null;

    //Where all Sensors are defined
    public ModernRoboticsI2cGyro modernRoboticsGyroSensor = null;
   // public LightSensor legoLineSensor = null;
   // public ModernRoboticsI2cRangeSensor rangeSensor = null;

    //1000 Milliseconds
    public int SECOND = 1000;

    //Ultra Sonic Distance
    int ultraSonicDistance = 11;

    //ShooterMotor Variables
    public int ROTATION = 1220; // # of ticks
    public int MOTOR_POWER = 1;

    //Z-Axis of the Modern Robotics Gyro Sensor
    int heading = 0;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor *///Empty Constructor
    public RobotHardware()
    {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap)
    {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontRight = hwMap.dcMotor.get("motor_2");
        backRight = hwMap.dcMotor.get("motor_4");
        frontLeft = hwMap.dcMotor.get("motor_3");
        backLeft = hwMap.dcMotor.get("motor_1");

        //Define and Initialize Sensors
        modernRoboticsGyroSensor = hwMap.get(ModernRoboticsI2cGyro.class, "modernRoboticsGyroSensor");
       // legoLineSensor = hwMap.lightSensor.get("legoLineSensor");
       // rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");

        //Configure the Direction of the Motors
        frontRight.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        backRight.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Calibrate the Modern Robotics Gyro Sensor
        modernRoboticsGyroSensor.calibrate();

        //Turn on the LED of the Lego Line Sensor
        //legoLineSensor.enableLed(true);

        /*This prevents the Modern Robotics Gyro Sensor from
          incorrectly calibrating before the start of Autonomous
        */

        while (modernRoboticsGyroSensor.isCalibrating())
        {
            try
            {
                Thread.sleep(50);
            }

            catch (InterruptedException e)
            {
                //do nothing
            }
        }

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
            try
            {
                Thread.sleep(remaining);
            } catch (InterruptedException e)
            {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    //This function sets the motors to 0 stopping the Robot
    public void stopRobot()
    {
        frontRight.setPower(MOTOR_POWER * 0);
        backRight.setPower(MOTOR_POWER * 0);
        frontLeft.setPower(MOTOR_POWER * 0);
        backLeft.setPower(MOTOR_POWER * 0);
    }

    //A basic go straight function that stops after a certain time
    public void goStraight(double Speed, long Time) throws InterruptedException
    {
        frontRight.setPower(MOTOR_POWER * Speed);
        backRight.setPower(MOTOR_POWER * Speed);
        frontLeft.setPower(MOTOR_POWER * Speed);
        backLeft.setPower(MOTOR_POWER * Speed);

        Thread.sleep(Time);
    }

    //A basic go straight function that uses encoders to track its distance
    public void drive(int Distance, double Speed) throws InterruptedException
    {
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setPower(MOTOR_POWER * Speed);
        backRight.setPower(MOTOR_POWER * Speed);
        frontLeft.setPower(MOTOR_POWER * Speed);
        backLeft.setPower(MOTOR_POWER * Speed);

        frontRight.setTargetPosition(Distance);
        backRight.setTargetPosition(Distance);
        frontLeft.setTargetPosition(Distance);
        backLeft.setTargetPosition(Distance);

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy())
        {

        }

        stopRobot();

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //A basic Turn function that uses the Modern Robotics Gyro Sensor to calculate the angle
    public void turnGyro(String Direction, int angle, double Speed) throws InterruptedException
    {
        int MotorDirectionChange = 0;

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        if (Direction.equals("Left"))
        {
            MotorDirectionChange = -1;
        }
        else
        if (Direction.equals("Right"))
        {
        MotorDirectionChange = 1;
        }

        while ((heading > angle + 5 || heading < angle - 2))
        {
            frontRight.setPower(MOTOR_POWER * Speed * MotorDirectionChange);
            backRight.setPower(MOTOR_POWER * Speed * MotorDirectionChange);

            frontLeft.setPower(-MOTOR_POWER * Speed * MotorDirectionChange);
            backLeft.setPower(-MOTOR_POWER * Speed * MotorDirectionChange);

            heading = modernRoboticsGyroSensor.getHeading();
        }

        stopRobot();

        }

    }

