package org.firstinspires.ftc.teamcode.Sam.shooter;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Sam.shooter.beans.ShooterMotor;
import org.firstinspires.ftc.teamcode.Sam.shooter.util.Constants;
import org.firstinspires.ftc.teamcode.Sam.shooter.util.MotorUtil;

public class RPMThread implements Runnable {

    private DcMotor shooter;
    private Enum motorName;

    //instance variables
    private long previousTime = System.nanoTime();
    private int previousPosition = 0;

    public RPMThread(DcMotor shooter, Enum motorName) {
        this.shooter = shooter;
        this.motorName = motorName;
    }

    @Override
    public void run() {
        ShooterMotor shooterMotor = MotorFactory.getInstance().getMotor(motorName);
        //DbgLog.msg("dt: " + (System.nanoTime() - previousTime));
        //deltaTime should be the same as in PowerManager
//        long deltaTime = System.nanoTime() - previousTime;
        double rpm = MotorUtil.getCurrentRPM(Constants.DELTA_TIME, shooter.getCurrentPosition(), previousPosition);
        shooterMotor.setRpm(rpm);

        previousTime = System.nanoTime();
        previousPosition = shooter.getCurrentPosition();
    }

}
