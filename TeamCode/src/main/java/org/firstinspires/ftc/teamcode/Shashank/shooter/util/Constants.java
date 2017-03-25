package org.firstinspires.ftc.teamcode.Shashank.shooter.util;


public final class Constants {
    private Constants() {
    }

    public enum MOTORNAME {
        LEFT_SHOOTER, RIGHT_SHOOTER
    }

    //28 ppr, or 28 encoder ticks per revolution
    //6600 rpm
    //maximum rpm should be 184800
    public static int REQUESTED_ETPS = 88704;//83160;//77616;//35150;//1855;//1590;//1750 good for close shots
    public static final int DELTA_TIME = 50;
    public static final int ONE_SECOND = 1000;

    public static double DEFAULT_POWER = 0.48;//0.45;//0.455;//0.42
    public static final double ALLOWED_POWER_DIFF = 0.3;//0.03;

    public static final boolean USE_TELEMETRY=true;



}
