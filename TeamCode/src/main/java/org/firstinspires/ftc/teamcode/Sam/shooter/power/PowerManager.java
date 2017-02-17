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
    private PIDAlgo pidAlgo = new PIDAlgo();
    private MotorTelemetry motorTelemetry = new MotorTelemetry();

    public PowerManager(Constants.MOTORNAME motorName, DcMotor dcMotor) {
        this.motorName = motorName;
        this.dcMotor = dcMotor;

        this.dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void setAllowedPowerDifference(double allowedPowerDifference) {
        this.allowedPowerDifference = allowedPowerDifference;
    }

    public void regulatePower() {
        ShooterMotor motor = MotorFactory.getInstance().getMotor(motorName);

        currentPower += pidAlgo.getAdjustment(motor.getRpm(), Constants.REQUESTED_ETPS, Constants.DELTA_TIME);
        currentPower = clipPower(currentPower);

        motorTelemetry.setCurrentRpm(motor.getRpm());
        motorTelemetry.setKp(pidAlgo.getProportionAdjust());
        motorTelemetry.setKi(pidAlgo.getIntegralAdjust());
        motorTelemetry.setKd(pidAlgo.getDerivativeAdjust());
        motorTelemetry.setKalminX(pidAlgo.getKalminFilteredData());
        motorTelemetry.setKalminP(pidAlgo.getKalminPrevError());
        motorTelemetry.setKk(pidAlgo.getKalminTrustVal());
        motorTelemetry.setRequiredPwr(currentPower);

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
}





