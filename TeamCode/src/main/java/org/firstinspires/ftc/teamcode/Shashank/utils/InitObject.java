package org.firstinspires.ftc.teamcode.Shashank.utils;

/**
 * Created by spmeg on 4/15/2017.
 */

public class InitObject<String, T> {
    private T t;
    private String hardwareMapName;

    public InitObject(T t, String hardwareMapName) {
        this.t = t;
        this.hardwareMapName = hardwareMapName;
    }

    public T getT() {
        return t;
    }

    public String getHardwareMap() {
        return hardwareMapName;
    }
}
