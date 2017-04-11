package org.firstinspires.ftc.teamcode.Shashank.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Sam.shooter.RPMThreadMilliseconds;
import org.firstinspires.ftc.teamcode.Sam.shooter.util.Constants;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import trclib.TrcKalmanFilter;

/**
 * Created by spmeg on 4/1/2017.
 */

public class TwoControllerTeleopv7Shashank extends OpMode {

    private ScheduledExecutorService scheduledThreadPool = Executors.newScheduledThreadPool(200);
    private volatile double leftFilteredRPM;
    private volatile double rightFilteredRPM;
    TrcKalmanFilter leftKalamFilter = new TrcKalmanFilter("teleop v7", 0D, 200D);
    TrcKalmanFilter rightKalamFilter = new TrcKalmanFilter("teleop v7", 0D, 200D);

    @Override
    public void init() {
        for(int i = 0; i <  200; i++){
            //run a thread every fifty milliseconds, and each thread will re-run after a second
            //schedule the left threads
            scheduledThreadPool.scheduleAtFixedRate(new Runnable() {
                @Override
                public void run() {
                    leftFilteredRPM = leftKalamFilter.filterData(0.0);
                }
            }, i * 10, Constants.ONE_SECOND, TimeUnit.MILLISECONDS);

            //schedule the right threads
            scheduledThreadPool.scheduleAtFixedRate(new Runnable() {
                @Override
                public void run() {
                    rightFilteredRPM = rightKalamFilter.filterData(0.0);
                }
            }, i * 10, Constants.ONE_SECOND, TimeUnit.MILLISECONDS);
        }
    }

    @Override
    public void loop() {

    }
}
