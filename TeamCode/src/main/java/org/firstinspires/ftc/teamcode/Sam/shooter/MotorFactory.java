package org.firstinspires.ftc.teamcode.Sam.shooter;

import com.qualcomm.ftccommon.DbgLog;

import org.firstinspires.ftc.teamcode.Sam.shooter.beans.ShooterMotor;
import org.firstinspires.ftc.teamcode.Sam.shooter.util.Constants;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Semaphore;

/**
 * Is a Singleton used to store ShooterMotor attributes
 */

public class MotorFactory {

    private static MotorFactory factory = null;

    private MotorFactory() {
    }

    private static Semaphore mutex = new Semaphore(1);
    private Map<Constants.MOTORNAME, ShooterMotor> motors = new HashMap<Constants.MOTORNAME, ShooterMotor>();

    public static MotorFactory getInstance() {
        if (null == factory) {
            try {
                mutex.acquire();
                if (null == factory) {
                    factory = new MotorFactory();
                }
                mutex.release();
            } catch (InterruptedException e) {
                DbgLog.msg("Thread is interrupted");
                Thread.currentThread().interrupt();
            }
        }
        return factory;
    }


    public ShooterMotor getMotor(Constants.MOTORNAME name) {
        return motors.get(name);
    }

    public void addMotor(ShooterMotor shooterMotor) {
        motors.put(shooterMotor.getName(), shooterMotor);
    }


}
