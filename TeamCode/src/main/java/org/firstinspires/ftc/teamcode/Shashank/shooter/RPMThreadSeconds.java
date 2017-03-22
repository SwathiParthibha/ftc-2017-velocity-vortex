package org.firstinspires.ftc.teamcode.Shashank.shooter;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Shashank.shooter.beans.ShooterMotor;
import org.firstinspires.ftc.teamcode.Shashank.shooter.util.Constants;
import org.firstinspires.ftc.teamcode.Shashank.shooter.util.MotorUtil;

public class RPMThreadSeconds implements Runnable {

    private DcMotor shooter;
    private Constants.MOTORNAME motorName;

    //instance variables
    private int previousPosition = 0;

    public RPMThreadSeconds(DcMotor shooter, Constants.MOTORNAME motorName) {
        this.shooter = shooter;
        this.motorName = motorName;
    }

    @Override
    public void run() {
        DbgLog.msg("LOGGING RPM THREAD "+ this.toString());
        ShooterMotor shooterMotor = MotorFactory.getInstance().getMotor(motorName);
        int rpm = MotorUtil.getCurrentRPM(Constants.DELTA_TIME, shooter.getCurrentPosition(), previousPosition);
        shooterMotor.setRpm(rpm);
        previousPosition = shooter.getCurrentPosition();
    }

}
