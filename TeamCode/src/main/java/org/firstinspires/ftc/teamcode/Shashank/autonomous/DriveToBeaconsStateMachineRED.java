package org.firstinspires.ftc.teamcode.Shashank.autonomous;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Mrinali.OldAutonomous.HardwarePushbot;
import org.firstinspires.ftc.teamcode.Shashank.utils.IMUInitialization;

import java.util.Locale;

/**
 * Created by spmeg on 10/22/2016.
 */
@Autonomous(name = "DriveToBeaconsStateMachineRED", group = "Autonomous")
@Disabled
public class DriveToBeaconsStateMachineRED extends OpMode {
    private ColorSensor leftColorSensor;
    private ColorSensor rightColorSensor;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private ModernRoboticsI2cRangeSensor rangeSensor;

    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    // OpticalDistanceSensor

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    LightSensor lightSensor;      // Primary LEGO Light sensor,
    //lightSensor;   // Alternative MR ODS sensor
    static final double WHITE_THRESHOLD = 0.3;  // spans between 0.1 - 0.5 from dark to light
    static final double APPROACH_SPEED = 0.4;
    double DIST = 11;

    enum State{
        TO_WHITE_LINE,
        PUSH_BUTTON,
        TURN,
        END
    }

    private State state;

    @Override
    public void init() {
        rangeSensor = this.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");

        leftColorSensor  = hardwareMap.colorSensor.get("lcs");

        rightColorSensor = hardwareMap.colorSensor.get("rcs");
        I2cAddr i2cAddr = I2cAddr.create8bit(0x4c);
        leftColorSensor.setI2cAddress(i2cAddr);

        leftMotor = hardwareMap.dcMotor.get("l");
        rightMotor = hardwareMap.dcMotor.get("r");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //gyro.calibrate();

        /*while (gyro.isCalibrating())  {
            sleep(50);
        }*/

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

        imu = new IMUInitialization(hardwareMap, telemetry).getIMU();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        if(imu == null)
            telemetry.log().add("Value of imu is null");

        telemetry.log().add("Axes unit"+ angles.angleUnit.name());

        composeTelemetry();

        state = State.TURN;
    }

    @Override
    public void loop() {;

        telemetry.addData(">", "Press A & B to reset Heading.");
        telemetry.addData("Distance", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("cm in ultrasonic", rangeSensor.cmUltrasonic());
        telemetry.addData("left", String.format("a=%d r=%d g=%d b=%d", leftColorSensor.alpha(), leftColorSensor.red(), leftColorSensor.green(), leftColorSensor.blue()));
        telemetry.addData("right", String.format("a=%d r=%d g=%d b=%d", rightColorSensor.alpha(), rightColorSensor.red(), rightColorSensor.green(), rightColorSensor.blue()));
        telemetry.addData("verify", verify());
        telemetry.addData("Light Level", lightSensor.getLightDetected());
        telemetry.addData("time", this.time);
        telemetry.update();

        if(angles == null)
            telemetry.log().add("Value of angle is null");


        switch (state){
            case TO_WHITE_LINE:
                try {
                    toWhiteLine();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                leftMotor.setPower(0);
                rightMotor.setPower(0);
                telemetry.update();
                sleep(750);

                state = State.PUSH_BUTTON;
                break;
            case PUSH_BUTTON:
                sleep(750);

                telemetry.update();
                //approachBeacon();
                pushButton();
                telemetry.update();

                sleep(740);

                leftMotor.setPower(0);
                rightMotor.setPower(0);

                state = State.END;
                break;
            case TURN:
                telemetry.update();
                turn(-50);
                telemetry.update();
                state = State.END;
                break;
            case END:
                stop();
                break;
        }

        /*int targetAngleZ =  angleZ - 32;
        while (angleZ < targetAngleZ){
            leftMotor.setPower(0.3);
            rightMotor.setPower(-0.3);
            telemetry.update();
        }*/
        /*telemetry.update();
        // Turn right
        robot.leftMotor.setPower(-APPROACH_SPEED);
        sleep(750);
        robot.leftMotor.setPower(0);

        telemetry.update();
        approachBeacon();
        pushButton();

        telemetry.update();
        // Go backwards slightly
        robot.rightMotor.setPower(-APPROACH_SPEED);
        robot.leftMotor.setPower(-APPROACH_SPEED);
        sleep(200);

        telemetry.update();
        // Turn left - parallel to wall
        robot.rightMotor.setPower(-APPROACH_SPEED);
        robot.leftMotor.setPower(APPROACH_SPEED);
        sleep(750); //REPLACE: Use gyro

        telemetry.update();
        robot.leftMotor.setPower(APPROACH_SPEED);
        robot.rightMotor.setPower(APPROACH_SPEED);
        sleep(750);

        telemetry.update();
        try {
            toWhiteLine();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        telemetry.update();
        //Turn right - to beacon
        robot.rightMotor.setPower(APPROACH_SPEED);
        robot.leftMotor.setPower(-APPROACH_SPEED);
        sleep(750); //REPLACE: Use gyro

        telemetry.update();
        approachBeacon();
        pushButton();

        telemetry.update();*/
    }

    void toWhiteLine() throws InterruptedException {
        // Start the robot moving forward, and then begin looking for a white line.
        robot.leftMotor.setPower(APPROACH_SPEED);
        robot.rightMotor.setPower(APPROACH_SPEED);

        // run until the white line is seen OR the driver presses STOP;
        while (lightSensor.getLightDetected() < WHITE_THRESHOLD) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();
        }

        telemetry.addData("Distance", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.update();

        // Stop all motors

        sleep(100);
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    void approachBeacon() {
        // Drive to set distance away, slow down, stop at set distance
        if (rangeSensor.getDistance(DistanceUnit.CM) > DIST * 3) {
            robot.leftMotor.setPower(APPROACH_SPEED);
            robot.rightMotor.setPower(APPROACH_SPEED);
        }

        while (rangeSensor.getDistance(DistanceUnit.CM) > DIST * 3) {

            telemetry.addData("Distance", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();

        }

        //Momentarily stop
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        sleep(200);

        if (rangeSensor.getDistance(DistanceUnit.CM) > DIST) {
            robot.leftMotor.setPower(APPROACH_SPEED * .25);
            robot.rightMotor.setPower(APPROACH_SPEED * .25);
        }

        while (rangeSensor.getDistance(DistanceUnit.CM) > DIST) {

            telemetry.addData("Distance", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();

        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    void turn(int degrees){
        //right is negative, left is positive
        double targetDegrees = angles.firstAngle + degrees;
        telemetry.addData("target Degrees", targetDegrees);
         if (targetDegrees > 360) {
            targetDegrees -= 360;
        } else if (targetDegrees < 0) {
            targetDegrees += 360;
        }
        telemetry.update();
        Telemetry.Item item = telemetry.addData("target Degrees", targetDegrees);
        Telemetry.Item item2 = telemetry.addData("Degrees", angles.firstAngle);
        telemetry.update();
        while(angles.firstAngle == targetDegrees){
            if(targetDegrees < 0){
                //if degrees is negative turn right
                leftMotor.setPower(0.3);
                rightMotor.setPower(-0.3);
            } else {
                leftMotor.setPower(-0.3);
                rightMotor.setPower(0.3);
            }
            telemetry.update();
        }

        //telemetry.removeItem(item);
    }

    void pushButton() {
        // Pushes button, then straightens
        // REPLACE: Code to push button, use color sensor

        leftColorSensor.enableLed(true);
        rightColorSensor.enableLed(true);

        int leftRed = leftColorSensor.red();
        int leftBlue = leftColorSensor.blue();
        int rightRed = rightColorSensor.red();
        int rightBlue = rightColorSensor.blue();

        while (!verify()){

            if(leftRed > rightRed && !verify()){
                //write the code here to press the left button
                leftMotor.setPower(0.3);
                rightMotor.setPower(0.0);

                //wait three seconds
                verify();
            } else if(rightRed > leftRed && !verify()){
                //write the code here to press the right button
                rightMotor.setPower(0.3);
                leftMotor.setPower(0.0);
                verify();
            } else{
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    private boolean verify() {
        if(leftColorSensor.argb() == 0 || rightColorSensor.argb() == 0)
            return false;

        if(leftColorSensor.argb() == 255 || rightColorSensor.argb() == 255)
            return false;

        if(Math.abs(leftColorSensor.red() - rightColorSensor.red()) < 2){
            return true;
        }

        return false;
    }

    private void sleep(int time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.EXTRINSIC).toAxesOrder(AxesOrder.ZYX);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
