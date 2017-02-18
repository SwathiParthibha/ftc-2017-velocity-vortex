package org.firstinspires.ftc.teamcode.Sam.shooter.util;


public final class MotorUtil {
    private MotorUtil() {
    }

    public static double getCurrentRPM(long deltaTime, int currentPosition, int prevPosition) {
        return 1000*((double) (currentPosition - prevPosition) / (double) deltaTime);
    }


}
