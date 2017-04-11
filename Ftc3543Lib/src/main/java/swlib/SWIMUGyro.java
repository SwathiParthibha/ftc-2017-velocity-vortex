/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package swlib;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import ftclib.FtcOpMode;
import trclib.TrcDbgTrace;
import trclib.TrcFilter;
import trclib.TrcGyro;
import trclib.TrcUtil;

/**
 * This class implements the Modern Robotics gyro extending TrcGyro. It provides implementation of the abstract
 * methods in TrcGyro. The Modern Robotics gyro supports 3 axes: x, y and z. It provides rotation rate data for
 * all 3 axes. However, it only provides heading data for the z-axis and the heading data is wrap-around.
 */
public class SWIMUGyro extends TrcGyro
{
    private static final String moduleName = "FtcMRGyro";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private BNO055IMU gyro;
    private double xRateData = 0.0;
    private double yRateData = 0.0;
    private double zRateData = 0.0;
    private double zHeadingData = 0.0;
    private boolean enabled = true;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param filters specifies an array of filters to use for filtering sensor noise, one for each axis. Since we
     *                have 3 axes, the array should have 3 elements. If no filters are used, it can be set to null.
     */
    public SWIMUGyro(HardwareMap hardwareMap, String instanceName, TrcFilter[] filters)
    {
        super(instanceName, 3, GYRO_HAS_X_AXIS | GYRO_HAS_Y_AXIS | GYRO_HAS_Z_AXIS, filters);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        gyro = hardwareMap.get(BNO055IMU.class, "imu");
    }   //FtcMRGyro

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param filters specifies an array of filters to use for filtering sensor noise, one for each axis. Since we
     *                have 3 axes, the array should have 3 elements. If no filters are used, it can be set to null.
     */
    public SWIMUGyro(String instanceName, TrcFilter[] filters)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, filters);
    }   //FtcMRGyro

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public SWIMUGyro(String instanceName)
    {
        this(instanceName, null);
    }   //FtcMRGyro

    /**
     * This method calibrates the sensor.
     */
    public void calibrate(BNO055IMU.Parameters parameters)
    {
        final String funcName = "calibrate";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        gyro.initialize(parameters);
        while (!gyro.isGyroCalibrated())
        {
            TrcUtil.sleep(10);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //calibrate

    public void calibrate(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        calibrate(parameters);
    }

    /**
     * This method check if the range sensor is enabled.
     *
     * @return true if the device state indicates it is enabled, false otherwise.
     */
    public boolean isDeviceEnabled()
    {
        final String funcName = "isDeviceEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(enabled));
        }

        return enabled;
    }   //isDeviceEnabled

    /**
     * This method is called to enable/disable the sensor so it is not hogging I2c bus bandwidth when not in use.
     *
     * @param enabled specifies true if enabling, false otherwise.
     */
    public void setDeviceEnabled(boolean enabled)
    {
        final String funcName = "setDeviceEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.enabled = enabled;
    }   //setDeviceEnabled

    //
    // Overriding TrcGyro methods.
    //

    /**
     * This method overrides the TrcGyro class. It doesn't have an x-integrator.
     */
    @Override
    public void resetXIntegrator()
    {
        final String funcName = "resetXIntegrator";

        //do nothing

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //resetXIntegrator

    /**
     * This method overrides the TrcGyro class. It doesn't have an y-integrator.
     */
    @Override
    public void resetYIntegrator()
    {
        final String funcName = "resetYIntegrator";

        //do nothing

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //resetYIntegrator

    /**
     * This method overrides the TrcGyro class and calls its own.
     */
    @Override
    public void resetZIntegrator()
    {
        final String funcName = "resetZIntegrator";

        //do nothing

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //resetZIntegrator

    //
    // Implements TrcGyro abstract methods.
    //

    /**
     * This method returns the raw data of the specified type for the x-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the x-axis.
     */
    @Override
    public SensorData<Double> getRawXData(DataType dataType)
    {
        final String funcName = "getRawXData";

        //
        // MR gyro supports only rotation rate for the x-axis.
        //
        if (dataType == DataType.ROTATION_RATE)
        {
            xRateData = gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).thirdAngle;
        }
        SensorData<Double> data = new SensorData<>(TrcUtil.getCurrentTime(), xRateData);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp:%.3f,value:%f", data.timestamp, data.value);
        }

        return data;
    }   //getRawXData

    /**
     * This method returns the raw data of the specified type for the y-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the y-axis.
     */
    @Override
    public SensorData<Double> getRawYData(DataType dataType)
    {
        final String funcName = "getRawYData";

        //
        // MR gyro supports only rotation rate for the x-axis.
        //
        if (dataType == DataType.ROTATION_RATE)
        {
            yRateData = gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).thirdAngle;
        }
        SensorData<Double> data = new SensorData<>(TrcUtil.getCurrentTime(), yRateData);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp:%.3f,value:%f", data.timestamp, data.value);
        }

        return data;
    }   //getRawYData

    /**
     * This method returns the raw data of the specified type for the z-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the z-axis.
     */
    @Override
    public SensorData<Double> getRawZData(DataType dataType)
    {
        final String funcName = "getRawZData";
        double value = 0.0;

        if (dataType == DataType.ROTATION_RATE)
        {
            zRateData = gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;
            value = zRateData;
        }
        else if (dataType == DataType.HEADING)
        {
            zHeadingData = -gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;
            value = zHeadingData;
        }
        SensorData<Double> data = new SensorData<>(TrcUtil.getCurrentTime(), value);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                               "=(timestamp:%.3f,value:%f", data.timestamp, data.value);
        }

        return data;
    }   //getRawZData

}   //class FtcMRGyro
