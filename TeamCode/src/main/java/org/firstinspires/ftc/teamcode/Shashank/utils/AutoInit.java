package org.firstinspires.ftc.teamcode.Shashank.utils;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceImpl;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by spmeg on 4/15/2017.
 */

public class AutoInit {
    private OpMode opMode;
    private List opModeInitObjectsList = new ArrayList<>();

    public AutoInit(OpMode opMode) {
        this.opMode = opMode;
    }

    public void init(){
        Class opModeClass = opMode.getClass();
        Method[] opModeMethods = opModeClass.getMethods();
        for(Method method: opModeMethods) {
            if (method.getReturnType().getName().contains("autoInit")) {
                //it is one of the methods that will help me auto init
                String methodName = method.getReturnType().getName();

                String fieldName = methodName.substring(7, methodName.length());
                Field field = null;
                try {
                    field = opModeClass.getField(fieldName);
                } catch (NoSuchFieldException e) {
                    e.printStackTrace();
                }

                try {
                    if(field.get(opMode).equals(null))
                        continue;
                } catch (IllegalAccessException e) {
                    e.printStackTrace();
                }

                try {
                    initialize(field, (String) method.getReturnType().getField("hardwareNameName").get(method.invoke(opMode)));
                } catch (NoSuchFieldException e) {
                    e.printStackTrace();
                } catch (InvocationTargetException e) {
                    e.printStackTrace();
                } catch (IllegalAccessException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    private void initialize(Field field, String hardwareName){
        try {
            if(field.getType().equals(DcMotor.class)) {
                field.set(opMode.hardwareMap.dcMotor.get(hardwareName), opMode);
            } else if (field.getType().equals(DcMotor.class)) {
                field.set(opMode.hardwareMap.dcMotor.get(hardwareName), opMode);
            } else if (field.getType().equals(ColorSensor.class)) {
                field.set(opMode.hardwareMap.colorSensor.get(hardwareName), opMode);
            } else if (field.getType().equals(OpticalDistanceSensor.class)) {
                field.set(opMode.hardwareMap.opticalDistanceSensor.get(hardwareName), opMode);
            } else if (field.getType().equals(BNO055IMU.class)) {
                field.set(opMode.hardwareMap.dcMotor.get(hardwareName), opMode);
            } else if (field.getType().equals(Servo.class)) {
                field.set(opMode.hardwareMap.servo.get(hardwareName), opMode);
            } else if (field.getType().equals(DeviceInterfaceModule.class)) {
                field.set(opMode.hardwareMap.deviceInterfaceModule.get(hardwareName), opMode);
            } else if (field.getType().equals(ModernRoboticsI2cGyro.class)) {
                field.set(opMode.hardwareMap.dcMotor.get(hardwareName), opMode);
            } else if (field.getType().equals(I2cDevice.class)) {
                field.set(opMode.hardwareMap.i2cDevice.get(hardwareName), opMode);
            } else if (field.getType().equals(I2cDeviceImpl.class)) {
                field.set(opMode.hardwareMap.i2cDevice.get(hardwareName), opMode);
            } else if (field.getType().equals(I2cDeviceSynch.class)) {
                field.set(opMode.hardwareMap.i2cDeviceSynch.get(hardwareName), opMode);
            } else if (field.getType().equals(I2cDeviceSynchImpl.class)) {
                field.set(opMode.hardwareMap.i2cDeviceSynch.get(hardwareName), opMode);
            }
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
    }
}