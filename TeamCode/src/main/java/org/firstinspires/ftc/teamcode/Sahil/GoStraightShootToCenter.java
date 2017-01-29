package org.firstinspires.ftc.teamcode.Sahil;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by SahilDoshi on 1/20/17.
 */

@Autonomous(name="GoStraightShootToCenter", group="Pushbot")
public class GoStraightShootToCenter extends LinearOpMode {

    //To change red to blue: negative angles, color sensors sense blue, right side range sensor
    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    private DcMotor shooter1;
    private DcMotor shooter2;
    private boolean state;
    private DcMotor scooper;
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
    double COUNTS_PER_INCH = ROTATION /
            (WHEEL_SIZE_IN * Math.PI);
    double DIST = 18;
    double SIDE_DIST = 30;
    byte[] rangeSensorCache;
    byte[] sideRangeSensorCache;
    I2cDevice rangeA;
    I2cDevice rangeB;
    public class shooterSettings{//data members can be replaced, but default values are for 1750 ETPS = 955 RPM

        public shooterSettings(){}
        public shooterSettings(double therequestedRPM, double theoriginalPWR1, double theoriginalPWR2){
            requestedRPM=therequestedRPM;
            requestedEncoderTicksPerSecond =requestedRPM*110/60;
            originalPWR1=theoriginalPWR1;
            originalPWR2=theoriginalPWR2;
            requiredPWR1=originalPWR1;
            requiredPWR2=originalPWR2;
        }

        private double requestedRPM =955;//955;
        private double requestedEncoderTicksPerSecond =requestedRPM*110/60;//1750

        //PID variables
        private double dt=0;
        private double previous_position1=0;
        private double current_position1=0;
        private double current_rpm1=0;
        private double previous_rpm1=0;
        private double error1=0;
        private double previous_error1=0;
        private double integral1=0;
        private double derivative1=0;
        private double adjustment1=0;
        private double previous_position2=0;
        private double current_position2=0;
        private double current_rpm2=0;
        private double previous_rpm2=0;
        private double error2=0;
        private double previous_error2=0;
        private double integral2=0;
        private double derivative2=0;
        private double adjustment2=0;

        //PID Constants
        double Kp = 0.000001;
        double Ki = 0.0000001;//0.00000001
        double Kd = 0.0000001;

        //Timing variables
        public double rampUpTime=1.5;

        //Power Variables
        public double originalPWR1=0.42;
        public double originalPWR2=0.42;
        public final double allowedPowerDifference=0.03;
        public double requiredPWR1=originalPWR1;
        public double requiredPWR2=originalPWR2;
        public double deadband=20;





        //Kalman Filter Variables
        double input1=0;
        double prevXk1=0;
        double prevPk1=1;
        double Xk1=0;
        double Pk1=1;
        double Kk1=0;
        double R1=0.2;

        double input2=0;
        double prevXk2=0;
        double prevPk2=1;
        double Xk2=0;
        double Pk2=1;
        double Kk2=0;
        double R2=0.2;



    }

    private boolean USE_TELEMETRY=false;



    shooterSettings RPM955;
    shooterSettings RPM0;
    shooterSettings RPM800;



    public double startShootingtime=0;
    public double prevTime=0;


    public void EncoderShooter(shooterSettings settings)
    {
        if(settings.requestedRPM!=0) {


            if(startShootingtime==-999) {//only update on first run
                startShootingtime = getRuntime();
            }

            settings.dt=getRuntime()-prevTime;
            if (settings.dt> 0.01) {//only update every 10ms
                settings.current_position1 = shooter1.getCurrentPosition();//MUST BE FIRST - time sensitive measurement
                settings.current_position2 = shooter2.getCurrentPosition();//MUST BE FIRST - time sensitive measurement
                prevTime = getRuntime();//MUST BE FIRST - time sensitive measurement

                updateRPM1and2(settings);

                if(getRuntime()-startShootingtime>settings.rampUpTime) {//only update Kalmin and PID after ramp up
                    timeUpdate(settings);
                    measurementUpdate(settings);


                    //DbgLog.msg("Time: "+getRuntime()+"RPM1: " + current_rpm1+"RPM2: " + current_rpm2);

                    PID1Update(settings);
                    PID2Update(settings);

                    applyAdjustment1(settings);
                    applyAdjustment2(settings);
                }
                clipPower1(settings);
                clipPower2(settings);

                previous1Update(settings);
                previous2Update(settings);


            }

            checkIfReadyToShoot(settings);
            if(USE_TELEMETRY) {
                outputTelemetry(settings);
            }


            shooter1.setPower(settings.requiredPWR1);
            shooter2.setPower(settings.requiredPWR2);


        }
        else
        {
            shooter1.setPower(0);
            shooter2.setPower(0);
            startShootingtime=-999;
            resetKalmin(settings);
            resetPID(settings);
        }

    }


    public void updateRPM1and2(shooterSettings settings){
        settings.current_rpm1 = (settings.current_position1 - settings.previous_position1) / (settings.dt);
        settings.current_rpm2 = (settings.current_position2 - settings.previous_position2) / (settings.dt);
    }

    public void PID1Update(shooterSettings settings){
        settings.error1=-(settings.Xk1- settings.requestedEncoderTicksPerSecond);
        settings.integral1 = settings.integral1 + settings.error1 * settings.dt;//calculate integral of error
        settings.derivative1 = (settings.error1 - settings.previous_error1) / settings.dt;//calculate derivative of data

        if(Math.abs(settings.error1)<settings.deadband)
        {
            settings.integral1=0;
            settings.derivative1=0;
        }

        settings.adjustment1 = settings.Kp * settings.error1 + settings.Kd*settings.derivative1 + settings.Ki*settings.integral1;// + Ki * integral1 + Kd * derivative1;//summation of PID



    }

    public void PID2Update(shooterSettings settings){

        settings.error2=-(settings.Xk2- settings.requestedEncoderTicksPerSecond);
        settings.integral2 = settings.integral2 + settings.error2 * settings.dt;//calculate integral of error
        settings.derivative2 = (settings.error2 - settings.previous_error2) / settings.dt;//calculate derivative of data

        if(Math.abs(settings.error2)<settings.deadband)
        {
            settings.integral2=0;
            settings.derivative2=0;
        }

        settings.adjustment2 = settings.Kp * settings.error2 + settings.Kd*settings.derivative2 + settings.Ki*settings.integral2;// + Ki * integral1 + Kd * derivative1;//summation of PID


    }

    public void previous1Update(shooterSettings settings){
        settings.previous_error1=settings.error1;
        settings.previous_position1 = settings.current_position1;
        settings.previous_rpm1 = settings.current_rpm1;
    }

    public void previous2Update(shooterSettings settings){
        settings.previous_error2=settings.error2;
        settings.previous_position2 = settings.current_position2;
        settings.previous_rpm2 = settings.current_rpm2;
    }

    public void applyAdjustment1(shooterSettings settings) {
        settings.requiredPWR1+=settings.adjustment1;
    }

    public void applyAdjustment2(shooterSettings settings) {
        settings.requiredPWR2+=settings.adjustment2;
    }

    public void clipPower1(shooterSettings settings){
        if(settings.requiredPWR1<settings.originalPWR1-settings.allowedPowerDifference)
        {
            settings.requiredPWR1=settings.originalPWR1-settings.allowedPowerDifference;
        }
        else if(settings.requiredPWR1>settings.originalPWR1+settings.allowedPowerDifference)
        {
            settings.requiredPWR1=settings.originalPWR1+settings.allowedPowerDifference;
        }

    }

    public void clipPower2(shooterSettings settings){
        if(settings.requiredPWR2<settings.originalPWR2-settings.allowedPowerDifference)
        {
            settings.requiredPWR2=settings.originalPWR2-settings.allowedPowerDifference;
        }
        else if(settings.requiredPWR2>settings.originalPWR2+settings.allowedPowerDifference)
        {
            settings.requiredPWR2=settings.originalPWR2+settings.allowedPowerDifference;
        }

    }


    public boolean checkIfReadyToShoot(shooterSettings settings) {
        if(Math.abs(settings.error1)<settings.deadband && Math.abs(settings.error2)<settings.deadband && getRuntime()-startShootingtime>settings.rampUpTime)
        {
            telemetry.addData("READY TO SHOOT", "");
            return true;
        }
        else
        {
            return false;
        }

    }
    public void outputTelemetry(shooterSettings settings) {
        telemetry.addData("requiredPWR1: ", String.format("%.4f", settings.requiredPWR1));
        telemetry.addData("requiredPWR2: ", String.format("%.4f", settings.requiredPWR2));
        telemetry.addData("adjustment1: ", settings.adjustment1);
        telemetry.addData("P1: ", settings.Kp*settings.error1);
        telemetry.addData("I1: ", settings.Ki*settings.integral1);
        telemetry.addData("D1: ", settings.Kd*settings.derivative1);
        telemetry.addData("adjustment2: ", settings.adjustment2);
        telemetry.addData("P2: ", settings.Kp*settings.error2);
        telemetry.addData("I2: ", settings.Ki*settings.integral2);
        telemetry.addData("D2: ", settings.Kd*settings.derivative2);
        telemetry.addData("curr1", settings.current_rpm1);
        telemetry.addData("curr2", settings.current_rpm2);
        telemetry.addData("Kalmin1", settings.Xk1);
        telemetry.addData("Kalmin2", settings.Xk2);
        telemetry.addData("K1", settings.Kk1);
        telemetry.addData("K2", settings.Kk2);
        telemetry.addData("Time: ", "" + getRuntime());
        telemetry.addData("ReqestedETPS", settings.requestedEncoderTicksPerSecond);

    }

    //Kalmin phase 1
    public void timeUpdate(shooterSettings settings){
        settings.input1=settings.current_rpm1;
        settings.prevXk1=settings.Xk1;
        settings.prevPk1=settings.Pk1;

        settings.input2=settings.current_rpm2;
        settings.prevXk2=settings.Xk2;
        settings.prevPk2=settings.Pk2;
    }

    //Kalmin phase 2
    public void measurementUpdate(shooterSettings settings){
        //RPM1 calculations
        settings.Kk1=settings.prevPk1/(settings.prevPk1+settings.R1);
        settings.Xk1=settings.prevXk1+settings.Kk1*(settings.input1-settings.prevXk1);
        settings.Pk1=(1-settings.Kk1)*settings.prevPk1;

        //RPM2 calculations
        settings.Kk2=settings.prevPk2/(settings.prevPk2+settings.R2);
        settings.Xk2=settings.prevXk2+settings.Kk2*(settings.input2-settings.prevXk2);
        settings.Pk2=(1-settings.Kk2)*settings.prevPk2;


    }

    public void resetKalmin(shooterSettings settings){
        settings.input1=0;
        settings.prevXk1=0;
        settings.prevPk1=1;
        settings.Xk1=0;
        settings.Pk1=1;
        // Kk1=0;


        settings.input2=0;
        settings.prevXk2=0;
        settings.prevPk2=1;
        settings.Xk2=0;
        settings.Pk2=1;
        // Kk2=0;



    }

    public void resetPID(shooterSettings settings){
        settings.previous_position1=0;
        settings.current_position1=0;
        settings.current_rpm1=0;
        settings.previous_rpm1=0;
        settings.error1=0;
        settings.previous_error1=0;
        settings.integral1=0;
        settings.derivative1=0;
        settings.adjustment1=0;
        settings.previous_position2=0;
        settings.current_position2=0;
        settings.current_rpm2=0;
        settings.previous_rpm2=0;
        settings.error2=0;
        settings.previous_error2=0;
        settings.integral2=0;
        settings.derivative2=0;
        settings.adjustment2=0;


    }

    public void runOpMode() throws InterruptedException {

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        RPM955= new shooterSettings();//default settings are for 955, 0.43,0.43
        RPM0 = new shooterSettings(0,0,0);
        RPM800 = new shooterSettings(800,0.35,0.35);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1 = this.hardwareMap.dcMotor.get("shooter1");
        shooter2 = this.hardwareMap.dcMotor.get("shooter2");
        scooper = this.hardwareMap.dcMotor.get("scooper");

        state = false;

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {
            idle();
        }

        // sleep(10000);
        encoderDrive(APPROACH_SPEED, 20 / 2, 20 / 2, 3);
        shoot();
        encoderDrive(APPROACH_SPEED, 40 / 2, 40 / 2, 10);
    }



    private boolean verify() {
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



    public void drive(double distance, double speed) throws InterruptedException {
        //1220 ticks per rotation
        //how many rotations? depends on distance
        //distance in cm - convert distance to encoder ticks
        //distance for each encoder tick == circumference / 1220
        //target distance / distance for each encoder tick == number of encoder ticks needed
        telemetry.addData("Starting to Drive", robot.leftMotor.getCurrentPosition() / ROTATION);
        telemetry.update();

        runUsingEncoder();

        stopAndResetEncoder();

        robot.leftMotor.setPower(speed);
        robot.rightMotor.setPower(speed);

        robot.leftMotor.setTargetPosition(CMtoEncoderTicks(distance));
        robot.rightMotor.setTargetPosition(CMtoEncoderTicks(distance));

        runToPosition();

        while (robot.leftMotor.isBusy() && robot.rightMotor.isBusy() && opModeIsActive()) {
            //telemetry.addData("Heading", heading);
        }

        stopRobot();

        runUsingEncoder();

        telemetry.addData("Finished Driving", robot.leftMotor.getCurrentPosition() / ROTATION);
        telemetry.update();
    }

    int CMtoEncoderTicks(double cm) {
        return (int) (cm * ROTATION / WHEEL_SIZE_IN / Math.PI);
        //(target dist * ticks per rotation) / (circumference)
    }

    public void stopRobot() {
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    public void stopAndResetEncoder() {
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runToPosition() {
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runUsingEncoder() {
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {

        ElapsedTime runtime = new ElapsedTime();
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.addData("Left motor busy", robot.leftMotor.isBusy());
                telemetry.addData("Right motor busy", robot.rightMotor.isBusy());
                telemetry.update();
            }
            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }
    public void shoot() {

        while(opModeIsActive())
        {
            EncoderShooter(RPM955);
            if(checkIfReadyToShoot(RPM955)){
                scooper.setPower(1);
                sleep(3000);
                break;
            }
        }

        telemetry.addData("Finish", "After");
    }


}
