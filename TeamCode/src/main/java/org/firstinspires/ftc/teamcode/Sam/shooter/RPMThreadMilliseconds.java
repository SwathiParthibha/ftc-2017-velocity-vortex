package org.firstinspires.ftc.teamcode.Sam.shooter;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Sam.shooter.beans.ShooterMotor;
import org.firstinspires.ftc.teamcode.Sam.shooter.util.Constants;
import org.firstinspires.ftc.teamcode.Sam.shooter.util.MotorUtil;

import java.util.concurrent.TimeUnit;

public class RPMThreadMilliseconds implements Runnable {

    private DcMotor shooter;
    private Constants.MOTORNAME motorName;

    //instance variables
    private int previousPosition = 0;

    public RPMThreadMilliseconds(DcMotor shooter, Constants.MOTORNAME motorName) {
        this.shooter = shooter;
        this.motorName = motorName;
    }

    @Override
    public void run() {
        ShooterMotor shooterMotor = MotorFactory.getInstance().getMotor(motorName);
        double rpm = MotorUtil.getCurrentRPM(Constants.DELTA_TIME, TimeUnit.MILLISECONDS, shooter.getCurrentPosition(), previousPosition);
        shooterMotor.setRpm(rpm);
        previousPosition = shooter.getCurrentPosition();
    }

}
