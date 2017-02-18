package org.firstinspires.ftc.teamcode.Sam.shooter.beans;

import com.qualcomm.ftccommon.DbgLog;

import org.firstinspires.ftc.teamcode.Sam.shooter.util.Constants;

public class ShooterMotor {
    private Constants.MOTORNAME name;
    private double rpm = 0D;

    public Constants.MOTORNAME getName() {
        return name;
    }

    public void setName(Constants.MOTORNAME name) {
        this.name = name;
    }

    public double getRpm() {
        //DbgLog.msg("rpm: "+rpm);
        return rpm;
    }

    public void setRpm(double rpm) {
        this.rpm = rpm;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        ShooterMotor shooterMotor = (ShooterMotor) o;

        return name != null ? name.equals(shooterMotor.name) : shooterMotor.name == null;

    }

    @Override
    public int hashCode() {
        return name != null ? name.hashCode() : 0;
    }
}
