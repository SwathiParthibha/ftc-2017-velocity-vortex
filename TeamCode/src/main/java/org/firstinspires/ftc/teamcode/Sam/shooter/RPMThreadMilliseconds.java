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
        //at 50% power, you should be reading around 3769920 encoder ticks per second
        ShooterMotor shooterMotor = MotorFactory.getInstance().getMotor(motorName);
        int rpm = shooterMotor.getRpm();
        if(shooter.getPower() > 0)
            rpm = MotorUtil.getCurrentRPM(Constants.ONE_SECOND, shooter.getCurrentPosition(), previousPosition);
        shooterMotor.setRpm(rpm);
        previousPosition = shooter.getCurrentPosition();
    }

}
