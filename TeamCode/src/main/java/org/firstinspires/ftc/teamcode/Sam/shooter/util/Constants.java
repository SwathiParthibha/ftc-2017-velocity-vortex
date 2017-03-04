package org.firstinspires.ftc.teamcode.Sam.shooter.util;


public final class Constants {
    private Constants() {
    }

    public enum MOTORNAME {
        LEFT_SHOOTER, RIGHT_SHOOTER
    }

    public static int REQUESTED_ETPS = 1855;//1590;//1750 good for close shots
    public static final int DELTA_TIME = 50;

    public static double DEFAULT_POWER = 0.48;//0.455;//0.42
    public static final double ALLOWED_POWER_DIFF = 0.03;

    public static final boolean USE_TELEMETRY=true;



}
