package org.firstinspires.ftc.teamcode.Shashank.utils;

/**
 * Created by spmeg on 2/10/2017.
 */

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mrinali.Shoot;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import static java.util.concurrent.Executors.*;

/**
 * Created by spmega on 1/5/17.
 */

public class RPMRunnableTest implements Runnable {
    private Telemetry telemetry = null;
    private ElapsedTime runtime = null;
    private boolean requestStop = false;

    private DcMotor shooter1;
    private DcMotor shooter2;

    private ThreadSharedObject threadSharedObject;

    private ShooterSettings settings;

    public RPMRunnableTest(DcMotor shooter1, DcMotor shooter2, ShooterSettings settings, Telemetry telemetry, ThreadSharedObject threadSharedObject, ElapsedTime runtime) {
        this.shooter1 = shooter1;
        this.shooter2 = shooter2;
        this.settings = settings;
        this.telemetry = telemetry;
        this.threadSharedObject = threadSharedObject;
        this.runtime = runtime;
    }

    private boolean USE_TELEMETRY=true;

    public double startShootingtime=0;
    public double prevTime=0;

    public static final String SHOOTER_1_TAG = "RPM_SHOOTER_1";
    public static final String SHOOTER_2_TAG = "RPM_SHOOTER_2";

    private boolean scheduledRpmTask = false;

    @Override
    public void run() {
        ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
        settings.dt=50000000L;
        //if you need to run a loop, make sure that you check that requestStop is still false
        while (!requestStop) {
            //simply printing out and updating the telemetry log here

            if(settings.requestedRPM!=0){
                if(!scheduledRpmTask){
                    executor.schedule(new Runnable() {
                        @Override
                        public void run() {
                            DbgLog.msg("Nano Time: "+ settings.dt);
                            settings.current_position1 = shooter1.getCurrentPosition();//MUST BE FIRST - time sensitive measurement
                            settings.current_position2 = shooter2.getCurrentPosition();//MUST BE FIRST - time sensitive measurement

                            updateRPM1and2(settings);

                            DbgLog.msg("RPM1: " + settings.current_rpm1+"\nRPM2: " + settings.current_rpm2);

                            if(getRuntime()-startShootingtime>settings.rampUpTime) {//only update Kalmin and PID after ramp up
                                //timeUpdate(settings);
                                //measurementUpdate(settings);


                                //DbgLog.msg("Time: "+getRuntime()+"RPM1: " + settings.current_rpm1+"RPM2: " + settings.current_rpm2+"Kalmin1: " + settings.Xk1+"Kalmin2: " + settings.Xk2);
                                //DbgLog.msg("Time: "+getRuntime()+"Encoder: " + settings.current_position2+"Encoder2: " + settings.current_position2);

                                PID1Update(settings);
                                PID2Update(settings);

                                applyAdjustment1(settings);
                                applyAdjustment2(settings);
                            }
                            //clipPower1(settings);
                            //clipPower2(settings);

                            previous1Update(settings);
                            previous2Update(settings);
                            scheduledRpmTask = false;
                        }
                    }, (long) settings.dt, TimeUnit.NANOSECONDS);
                    scheduledRpmTask = true;
                }

                checkIfReadyToShoot(settings);
                if(USE_TELEMETRY) {
                    outputTelemetry(settings);
                }

                if(Double.isNaN(settings.requiredPWR1)){
                    settings.requiredPWR1 = 0;
                    settings.requiredPWR2 = 0;
                }

                shooter1.setPower(Range.clip(settings.requiredPWR1, 0, 1));
                shooter2.setPower(Range.clip(settings.requiredPWR2, 0, 1));
            } else
            {
                shooter1.setPower(0);
                shooter2.setPower(0);
                startShootingtime=-999;
                resetKalmin(settings);
                resetPID(settings);
            }

            /*if(settings.requestedRPM!=0){
                settings.dt=System.nanoTime()-prevTime;
                if (settings.dt>= 50000000L) {//only update every 50ms
                    settings.current_position1 = shooter1.getCurrentPosition();//MUST BE FIRST - time sensitive measurement
                    settings.current_position2 = shooter2.getCurrentPosition();//MUST BE FIRST - time sensitive measurement
                    prevTime = System.nanoTime();//MUST BE FIRST - time sensitive measurement

                    updateRPM1and2(settings);

                    // TODO: 2/4/2017 REPLACE THE RUNTIME WITH A DIFFERENT TIME MEASUREMENT
                    if(getRuntime()-startShootingtime>settings.rampUpTime) {//only update Kalmin and PID after ramp up
                        //timeUpdate(settings);
                        //measurementUpdate(settings);


                        //DbgLog.msg("Time: "+getRuntime()+"RPM1: " + settings.current_rpm1+"RPM2: " + settings.current_rpm2+"Kalmin1: " + settings.Xk1+"Kalmin2: " + settings.Xk2);
                        //DbgLog.msg("Time: "+getRuntime()+"Encoder: " + settings.current_position2+"Encoder2: " + settings.current_position2);
                        DbgLog.msg("Nano Time: "+ settings.dt);
                        DbgLog.msg("RPM1: " + settings.current_rpm1+"\nRPM2: " + settings.current_rpm2);

                        PID1Update(settings);
                        PID2Update(settings);

                        applyAdjustment1(settings);
                        applyAdjustment2(settings);
                    }
                    //clipPower1(settings);
                    //clipPower2(settings);

                    previous1Update(settings);
                    previous2Update(settings);

                }


                checkIfReadyToShoot(settings);
                if(USE_TELEMETRY) {
                    outputTelemetry(settings);
                }

                shooter1.setPower(Range.clip(settings.requiredPWR1, 0, 1));
                shooter2.setPower(Range.clip(settings.requiredPWR2, 0, 1));
            } else
            {
                shooter1.setPower(0);
                shooter2.setPower(0);
                startShootingtime=-999;
                resetKalmin(settings);
                resetPID(settings);
            }*/


            threadSharedObject.setDouble(SHOOTER_1_TAG, settings.current_rpm1);
            threadSharedObject.setDouble(SHOOTER_2_TAG, settings.current_position2);

        }
        executor.shutdown();
        //if you are not running a loop, make sure to periodically check if the requestStop is still false
        telemetry.log().add("This is ending a thread: " + runtime.toString());
    }

    private double getRuntime(){
        return runtime.seconds();
    }

    public boolean checkIfReadyToShoot(ShooterSettings settings) {
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

    public void outputTelemetry(ShooterSettings settings) {
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


    public void setSettings(ShooterSettings settings) {
        this.settings = settings;
    }

    public void applyAdjustment1(ShooterSettings settings) {
        settings.requiredPWR1+=settings.adjustment1;
    }

    public void applyAdjustment2(ShooterSettings settings) {
        settings.requiredPWR2+=settings.adjustment2;
    }

    public void PID1Update(ShooterSettings settings){
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

    public void resetKalmin(ShooterSettings settings){
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

    public void resetPID(ShooterSettings settings){
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

    public void previous1Update(ShooterSettings settings){
        settings.previous_error1=settings.error1;
        settings.previous_position1 = settings.current_position1;
        settings.previous_rpm1 = settings.current_rpm1;
    }

    public void previous2Update(ShooterSettings settings){
        settings.previous_error2=settings.error2;
        settings.previous_position2 = settings.current_position2;
        settings.previous_rpm2 = settings.current_rpm2;
    }


    public void clipPower1(ShooterSettings settings){
        if(settings.requiredPWR1<settings.originalPWR1-settings.allowedPowerDifference)
        {
            settings.requiredPWR1=settings.originalPWR1-settings.allowedPowerDifference;
        }
        else if(settings.requiredPWR1>settings.originalPWR1+settings.allowedPowerDifference)
        {
            settings.requiredPWR1=settings.originalPWR1+settings.allowedPowerDifference;
        }

    }

    public void clipPower2(ShooterSettings settings){
        if(settings.requiredPWR2<settings.originalPWR2-settings.allowedPowerDifference)
        {
            settings.requiredPWR2=settings.originalPWR2-settings.allowedPowerDifference;
        }
        else if(settings.requiredPWR2>settings.originalPWR2+settings.allowedPowerDifference)
        {
            settings.requiredPWR2=settings.originalPWR2+settings.allowedPowerDifference;
        }

    }

    public void PID2Update(ShooterSettings settings){

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


    public void updateRPM1and2(ShooterSettings settings){
        settings.current_rpm1 = (10^9)*(settings.current_position1 - settings.previous_position1) / (settings.dt);
        settings.current_rpm2 = (10^9)*(settings.current_position2 - settings.previous_position2) / (settings.dt);
    }

    //call this method when you want to stop the thread
    public void requestStop(){
        requestStop = true;
    }
}
