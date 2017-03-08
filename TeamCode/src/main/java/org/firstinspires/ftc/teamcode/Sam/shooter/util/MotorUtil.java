package org.firstinspires.ftc.teamcode.Sam.shooter.util;


import java.util.concurrent.TimeUnit;

public final class MotorUtil {
    private MotorUtil() {
    }

    public static double getCurrentRPM(long deltaTime, TimeUnit unit, int currentPosition, int prevPosition) {
        switch (unit){
            case MILLISECONDS:
                return 1000*((double) (currentPosition - prevPosition) / (double) deltaTime);
            case SECONDS:
                return 60*((double) (currentPosition - prevPosition) / (double) deltaTime);
            default:
                return 1000*((double) (currentPosition - prevPosition) / (double) deltaTime);
        }
    }


}
