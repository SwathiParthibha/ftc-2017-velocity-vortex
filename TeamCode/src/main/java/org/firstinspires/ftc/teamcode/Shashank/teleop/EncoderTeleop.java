package org.firstinspires.ftc.teamcode.Shashank.teleop;

import android.media.MediaPlayer;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.AllianceColor;
import org.firstinspires.ftc.teamcode.Shashank.utils.RPMRunnable;
import org.firstinspires.ftc.teamcode.Shashank.utils.shooterSettings;
import org.firstinspires.ftc.teamcode.Shashank.utils.ThreadSharedObject;

/**
 * Created by spmeg on 10/22/2016.
 */
@TeleOp(name = "Two Controller Teleop Test Shashank", group = "Teleop")
public class EncoderTeleop extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor scooper;
    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor sweeper;

    private boolean state;
    boolean swap=false;

    private boolean USE_TELEMETRY=false;

    private ElapsedTime runtime = new ElapsedTime();

    shooterSettings RPM955;
    shooterSettings RPM0;
    shooterSettings RPM800;
    shooterSettings RPM1300;


    public double startShootingtime=0;
    public double prevTime=0;

    private MediaPlayer wrongBallSound = null, correctBallSound = null;
    private ColorSensor sweeperColorSensor;
    private AllianceColor beaconColor = null;

    private boolean ballSensed = false;

    private RPMRunnable rpmRunnable = null;

    private ThreadSharedObject threadSharedObject = new ThreadSharedObject();

    @Override
    public void init() {
        leftMotor = this.hardwareMap.dcMotor.get("l");
        rightMotor = this.hardwareMap.dcMotor.get("r");
        scooper = this.hardwareMap.dcMotor.get("scooper");
        shooter1 = this.hardwareMap.dcMotor.get("shooter1");
        shooter2 = this.hardwareMap.dcMotor.get("shooter2");
        sweeper = this.hardwareMap.dcMotor.get("sweeper");
        sweeperColorSensor = this.hardwareMap.colorSensor.get("colorLegacy");
        state = false;


        RPM955= new shooterSettings();//default settings are for 955, 0.43,0.43
        RPM0 = new shooterSettings(0,0,0);
        RPM800 = new shooterSettings(800,0.35,0.35);
        RPM1300 = new shooterSettings(1100,0.59,0.59);


        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        swap=true;

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wrongBallSound = MediaPlayer.create(this.hardwareMap.appContext, R.raw.police_siren);
        correctBallSound = MediaPlayer.create(this.hardwareMap.appContext, R.raw.super_mario_power_up);

        rpmRunnable = new RPMRunnable(shooter1, shooter2, RPM0, telemetry, threadSharedObject, runtime);
        new Thread(rpmRunnable).start();
    }

    @Override
    public void start() {
        runtime.reset();
        super.start();
    }

    @Override
    public void loop() {

        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;
        int shooting1= shooter1.getCurrentPosition();
        int shooting2= shooter2.getCurrentPosition();

        if(swap==true)
        {
            double temp=left;
            left=right;
            right=temp;
        }

        left=scaleInput(left);
        right=scaleInput(right);

        leftMotor.setPower(left);
        rightMotor.setPower(right);

        if(gamepad1.dpad_down){
            leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            swap=false;
        } else if(gamepad1.dpad_up){
            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            swap=true;
        }

        if(gamepad2.dpad_right){
            sweeper.setPower(0.7);
            scooper.setPower(1);
        }

        if(gamepad2.left_trigger > 0){
            scooper.setPower(-0.7);
        } else if(gamepad2.left_bumper){
            scooper.setPower(1);
        } else {
            scooper.setPower(0);
        }

        if(gamepad2.a){
            //EncoderShooter(RPM800);
            rpmRunnable.setSettings(RPM800);
        } else if(gamepad2.b) {
            EncoderShooter(RPM955);//0.6//0.7
            rpmRunnable.setSettings(RPM955);
            //power=0.7;
            //startrunnning=true;
        } else if(gamepad2.y) {
            //EncoderShooter(RPM1300);//0.6//0.7
            rpmRunnable.setSettings(RPM1300);
            //power=0.7;
            //startrunnning=true;
        }
        else {
            //EncoderShooter(RPM0);
            rpmRunnable.setSettings(RPM0);
        }


        if(beaconColor == null) {
            if ((sweeperColorSensor.red()-sweeperColorSensor.blue()) > 15) {
                beaconColor = AllianceColor.RED;
                telemetry.log().add("Beacon Color Set");
            } else if((sweeperColorSensor.blue()-sweeperColorSensor.red()) > 15){
                beaconColor = AllianceColor.BLUE;
                telemetry.log().add("Beacon Color Set");
            } else {
                beaconColor = null;
                telemetry.log().add("Beacon Color Not Set");
            }
        }

        if(gamepad2.right_bumper){
            sweeper.setPower(0.7);

        } else if(gamepad2.right_trigger > 0){
            sweeper.setPower(-0.7);
        } else {
            sweeper.setPower(0);

        }

        if(isWrongBall()){
            if(!wrongBallSound.isPlaying()){
                wrongBallSound.release();
                wrongBallSound = MediaPlayer.create(this.hardwareMap.appContext, R.raw.police_siren);
                wrongBallSound.start();
            }
            //sweeper.setPower(0.5);
        } else {
            wrongBallSound.stop();
            //sweeper.setPower(0);
        }


        if(isWrongBall()){
            if(!wrongBallSound.isPlaying()){
                wrongBallSound.release();
                wrongBallSound = MediaPlayer.create(this.hardwareMap.appContext, R.raw.police_siren);
                wrongBallSound.start();
            }
            //sweeper.setPower(0.5);
        } else {
            wrongBallSound.stop();
            //sweeper.setPower(0);
        }

        telemetry.addData("left joystick",  "%.2f", left);
        telemetry.addData("right joystick", "%.2f", right);
        telemetry.addData("Beacon Color", beaconColor);
        telemetry.addData("lego color sensor red:", sweeperColorSensor.red());
        telemetry.addData("lego color sensor blue:", sweeperColorSensor.blue());
        telemetry.addData("lego color sensor green:", sweeperColorSensor.green());
        telemetry.addData("lego color sensor argb:", sweeperColorSensor.argb());
        telemetry.update();
    }

    @Override
    public void stop() {
        rpmRunnable.requestStop();
        super.stop();
    }

    private boolean isWrongBall() {
        if(sweeperColorSensor.red() > 11 || sweeperColorSensor.blue() > 11){
            ballSensed = true;
            if(sweeperColorSensor.red() > sweeperColorSensor.blue() && beaconColor == AllianceColor.BLUE){
                return true;
            } else if(sweeperColorSensor.blue() > sweeperColorSensor.red() && beaconColor == AllianceColor.RED){
                return true;
            } else {
                return false;
            }
        } else {
            ballSensed = false;
            return false;
        }
    }

    public void EncoderShooter(shooterSettings settings)
    {
        if(settings.requestedRPM!=0) {


            if(startShootingtime==-999) {//only update on first run
                startShootingtime = getRuntime();
            }

            settings.dt=getRuntime()-prevTime;
            if (settings.dt> 0.05) {//only update every 50ms
                settings.current_position1 = shooter1.getCurrentPosition();//MUST BE FIRST - time sensitive measurement
                settings.current_position2 = shooter2.getCurrentPosition();//MUST BE FIRST - time sensitive measurement
                prevTime = getRuntime();//MUST BE FIRST - time sensitive measurement

                updateRPM1and2(settings);

                if(getRuntime()-startShootingtime>settings.rampUpTime) {//only update Kalmin and PID after ramp up
                    //timeUpdate(settings);
                    //measurementUpdate(settings);


                    //DbgLog.msg("Time: "+getRuntime()+"RPM1: " + settings.current_rpm1+"RPM2: " + settings.current_rpm2+"Kalmin1: " + settings.Xk1+"Kalmin2: " + settings.Xk2);
                    DbgLog.msg("Time: "+getRuntime()+"Encoder: " + settings.current_position2+"Encoder2: " + settings.current_position2);

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
        //settings.error1=-(settings.Xk1- settings.requestedEncoderTicksPerSecond);
        settings.error1=-(settings.current_rpm1- settings.requestedEncoderTicksPerSecond);
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

        //settings.error2=-(settings.Xk2- settings.requestedEncoderTicksPerSecond);
        settings.error2=-(settings.current_rpm2- settings.requestedEncoderTicksPerSecond);
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

















    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }


}



