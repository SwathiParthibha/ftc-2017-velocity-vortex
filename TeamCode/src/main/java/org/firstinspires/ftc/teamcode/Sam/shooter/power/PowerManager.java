package org.firstinspires.ftc.teamcode.Sam.shooter.power;


import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Sam.shooter.MotorFactory;
import org.firstinspires.ftc.teamcode.Sam.shooter.beans.ShooterMotor;
import org.firstinspires.ftc.teamcode.Sam.shooter.util.Constants;

public class PowerManager {
    private double defaultPower = Constants.DEFAULT_POWER;
    private double currentPower = defaultPower;
    private double allowedPowerDifference = Constants.ALLOWED_POWER_DIFF;

    private DcMotor dcMotor;
    private Enum motorName;
    private PIDAlgo pidAlgo = new PIDAlgo();

    public PowerManager(Enum motorName, DcMotor dcMotor) {
        this.motorName = motorName;
        this.dcMotor = dcMotor;

        this.dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getDefaultPower() {
        return defaultPower;
    }

    public void setDefaultPower(double defaultPower) {
        this.defaultPower = defaultPower;
    }

    public double getCurrentPower() {
        return currentPower;
    }

    public double getAllowedPowerDifference() {
        return allowedPowerDifference;
    }

    public void setAllowedPowerDifference(double allowedPowerDifference) {
        this.allowedPowerDifference = allowedPowerDifference;
    }

    public void regulatePower() {
        ShooterMotor motor = MotorFactory.getInstance().getMotor(motorName);

        currentPower += pidAlgo.getAdjustment(motor.getRpm(), Constants.REQUESTED_ETPS, Constants.DELTA_TIME);
        currentPower = clipPower(currentPower);

        dcMotor.setPower(currentPower);
    }

    private double clipPower(double power) {
        if (power < defaultPower - allowedPowerDifference) {
            power = defaultPower - allowedPowerDifference;
        } else if (power > defaultPower + allowedPowerDifference) {
            power = defaultPower + allowedPowerDifference;
        }
        return power;
    }

}
