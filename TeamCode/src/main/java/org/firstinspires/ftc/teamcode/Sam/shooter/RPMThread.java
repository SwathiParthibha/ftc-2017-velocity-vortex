package org.firstinspires.ftc.teamcode.Sam.shooter;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Sam.shooter.beans.ShooterMotor;
import org.firstinspires.ftc.teamcode.Sam.shooter.util.Constants;
import org.firstinspires.ftc.teamcode.Sam.shooter.util.MotorUtil;

public class RPMThread implements Runnable {

    private DcMotor shooter;
    private Constants.MOTORNAME motorName;

    //instance variables
    private int previousPosition = 0;

    public RPMThread(DcMotor shooter, Constants.MOTORNAME motorName) {
        this.shooter = shooter;
        this.motorName = motorName;
    }

    @Override
    public void run() {
        ShooterMotor shooterMotor = MotorFactory.getInstance().getMotor(motorName);
        double rpm = MotorUtil.getCurrentRPM(Constants.DELTA_TIME, shooter.getCurrentPosition(), previousPosition);
        shooterMotor.setRpm(rpm);
        previousPosition = shooter.getCurrentPosition();
    }

}
