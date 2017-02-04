package org.firstinspires.ftc.teamcode.Shashank.utils;

/**
 * Created by spmeg on 1/21/2017.
 */

public interface BasicBot {
    void drive(double power);

    void turn(int angle);

    void driveToPos(int targetPos);

    void driveTimeout(int time);

    void init();

}
