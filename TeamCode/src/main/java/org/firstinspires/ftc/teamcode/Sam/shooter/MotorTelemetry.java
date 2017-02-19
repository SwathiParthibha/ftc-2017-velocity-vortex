package org.firstinspires.ftc.teamcode.Sam.shooter;

import org.firstinspires.ftc.teamcode.Sam.shooter.util.Constants;

public class MotorTelemetry {
    private double currentRpm;
    private Constants.MOTORNAME motorName;
    private double kp;
    private double ki;
    private double kd;
    private double kk;
    private double pk;
    private double kalminX;

    public Constants.MOTORNAME getMotorName() {
        return motorName;
    }

    public void setMotorName(Constants.MOTORNAME motorName) {
        this.motorName = motorName;
    }

    public double getRequiredPwr() {
        return requiredPwr;
    }

    public void setRequiredPwr(double requiredPwr) {
        this.requiredPwr = requiredPwr;
    }

    private double requiredPwr;

    public double getKp() {
        return kp;
    }

    public void setKp(double kp) {
        this.kp = kp;
    }

    public double getKi() {
        return ki;
    }

    public void setKi(double ki) {
        this.ki = ki;
    }

    public double getKd() {
        return kd;
    }

    public void setKd(double kd) {
        this.kd = kd;
    }

    public double getKk() {
        return kk;
    }

    public void setKk(double kk) {
        this.kk = kk;
    }

    public double getPk() {
        return pk;
    }

    public void setPk(double pk) {
        this.pk = pk;
    }

    public double getKalminX() {
        return kalminX;
    }

    public void setKalminX(double kalminX) {
        this.kalminX = kalminX;
    }

    public double getKalminP() {
        return kalminP;
    }

    public void setKalminP(double kalminP) {
        this.kalminP = kalminP;
    }

    public double getTime() {
        return time;
    }

    public void setTime(double time) {
        this.time = time;
    }

    private double kalminP;
    private double time;


    public double getCurrentRpm() {
        return currentRpm;
    }

    public void setCurrentRpm(double currentRpm) {
        this.currentRpm = currentRpm;
    }

    @Override
    public String toString() {
        StringBuilder strBuffer = new StringBuilder();
        strBuffer.append("MotorName: ").append(this.motorName.toString()).append(System.lineSeparator());
        strBuffer.append("CurrentRpm: ").append(this.currentRpm).append(System.lineSeparator());
        strBuffer.append("Kp: ").append(this.kp).append(System.lineSeparator());
        strBuffer.append("Ki: ").append(this.ki).append(System.lineSeparator());
        strBuffer.append("Kd: ").append(this.kd).append(System.lineSeparator());
        strBuffer.append("Kk: ").append(this.kk).append(System.lineSeparator());
        strBuffer.append("Pk: ").append(this.pk).append(System.lineSeparator());
        strBuffer.append("KalminX: ").append(this.kalminX).append(System.lineSeparator());
        strBuffer.append("KalminP: ").append(this.kalminP).append(System.lineSeparator());
        strBuffer.append("RequiredPwr: ").append(this.requiredPwr).append(System.lineSeparator());

        return strBuffer.toString();





    }
}
