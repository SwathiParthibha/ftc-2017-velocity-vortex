package org.firstinspires.ftc.teamcode.Sam.shooter.power;


import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Sam.shooter.MotorFactory;
import org.firstinspires.ftc.teamcode.Sam.shooter.MotorTelemetry;
import org.firstinspires.ftc.teamcode.Sam.shooter.beans.ShooterMotor;
import org.firstinspires.ftc.teamcode.Sam.shooter.util.Constants;

public class PowerManager {
    private double defaultPower = Constants.DEFAULT_POWER;
    private double currentPower = defaultPower;
    private double allowedPowerDifference = Constants.ALLOWED_POWER_DIFF;

    private DcMotor dcMotor;
    private Constants.MOTORNAME motorName;
    private PIDAlgo pidAlgo = null;
    private MotorTelemetry motorTelemetry = new MotorTelemetry();

    private double rpmErrorAdjustment = 0.0;
    private double powerAdjustment = 0.0;

    public PowerManager(Constants.MOTORNAME motorName, DcMotor dcMotor) {
        pidAlgo = new PIDAlgo(MotorFactory.getInstance().getMotor(motorName));
        this.motorName = motorName;
        this.dcMotor = dcMotor;

        this.dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //this.dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //this.dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTelemetry.setMotorName(motorName);
    }

    public Constants.MOTORNAME getMotorName() {
        return motorName;
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

    public double getRpmErrorAdjustment() {
        return rpmErrorAdjustment;
    }

    public double getPowerAdjustment() {
        return powerAdjustment;
    }

    public void setAllowedPowerDifference(double allowedPowerDifference) {
        this.allowedPowerDifference = allowedPowerDifference;
    }

    public void regulatePower() {
        ShooterMotor motor = MotorFactory.getInstance().getMotor(motorName);

        rpmErrorAdjustment = pidAlgo.getAdjustment(motor.getRpm(), Constants.REQUESTED_ETPS, Constants.ONE_SECOND);
        powerAdjustment = rpmErrorAdjustment/76;
        currentPower += powerAdjustment;
        currentPower = clipPower(currentPower);

        motorTelemetry.setCurrentRpm(motor.getRpm());
        motorTelemetry.setKp(pidAlgo.getProportionAdjust());
        motorTelemetry.setKi(pidAlgo.getIntegralAdjust());
        motorTelemetry.setKd(pidAlgo.getDerivativeAdjust());
        motorTelemetry.setKalminX(pidAlgo.getKalminFilteredData());
        motorTelemetry.setKalminP(pidAlgo.getKalminPrevError());
        motorTelemetry.setKk(pidAlgo.getKalminTrustVal());
        motorTelemetry.setRequiredPwr(currentPower);
        motorTelemetry.setRpmErrorAdjustment(rpmErrorAdjustment);
        motorTelemetry.setPowerAdjustment(powerAdjustment);

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

    public MotorTelemetry getMotorTelemetry() {
        return motorTelemetry;
    }

    public void reset()
    {
        pidAlgo.reset();
    }
}





