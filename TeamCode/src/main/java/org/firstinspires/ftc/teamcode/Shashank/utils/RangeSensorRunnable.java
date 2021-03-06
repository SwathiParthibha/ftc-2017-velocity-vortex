package org.firstinspires.ftc.teamcode.Shashank.utils;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by spmega on 1/5/17.
 */

public class RangeSensorRunnable implements Runnable {
    private Telemetry telemetry = null;
    private ElapsedTime runtime = null;
    private boolean requestStop = false;
    private I2cDeviceSynchImpl rangeSensor = null;
    private ThreadSharedObject threadSharedObject = null;

    private String ULTRASONIC_KEY = "";
    private String OPTICAL_KEY = "";

    public RangeSensorRunnable(Telemetry telemetry, ElapsedTime runtime, I2cDeviceSynchImpl rangeSensor, ThreadSharedObject threadSharedObject) {
        this.telemetry = telemetry;
        this.runtime = runtime;
        this.rangeSensor = rangeSensor;
        this.threadSharedObject = threadSharedObject;
        ULTRASONIC_KEY = rangeSensor.getDeviceName()+"ultrasonic";
        OPTICAL_KEY = rangeSensor.getDeviceName()+"optical";
    }

    public String getULTRASONIC_KEY() {
        return ULTRASONIC_KEY;
    }

    public String getOPTICAL_KEY() {
        return OPTICAL_KEY;
    }

    public void run() {
        while (!requestStop) {
            //telemetry.addData("This is printing from a thread: " ,runtime.toString());
            //telemetry.addData("From thread-RangeSensor Ultrasonic: ", getcmUltrasonic());
            //telemetry.addData("From thread-RangeSensor Optical: ", getOpticalDistance());
            //telemetry.update();
            threadSharedObject.setInteger(getULTRASONIC_KEY(), getcmUltrasonic());
            threadSharedObject.setInteger(getOPTICAL_KEY(), getOpticalDistance());
        }
        telemetry.log().add("This is ending a thread: " + runtime.toString());
    }

    private int getcmUltrasonic(){
        return rangeSensor.read(0x04, 2)[0]  & 0xFF;
    }

    private int getOpticalDistance() {
        return rangeSensor.read(0x04, 2)[1]  & 0xFF;
    }

    public void requestStop(){
        requestStop = true;
    }
}
