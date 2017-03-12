package org.firstinspires.ftc.teamcode.Sam.shooter.util;


import com.qualcomm.ftccommon.DbgLog;

import java.util.concurrent.TimeUnit;

public final class MotorUtil {
    private MotorUtil() {
    }

    public static int getCurrentRPM(int deltaTimeInMillis, int currentPosition, int prevPosition) {
        //one minute = 60000 milliseconds
        int conversion = 60000/deltaTimeInMillis;
        DbgLog.msg("the difference is " + (currentPosition - prevPosition));
        DbgLog.msg("conversion is: " + conversion);
        return conversion*(currentPosition - prevPosition);
    }


}
