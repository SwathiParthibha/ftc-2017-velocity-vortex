package org.firstinspires.ftc.teamcode.Sam.shooter.power;


public class PIDAlgo {
    private static final double DEAD_BAND = 20;
    private static final double KP = 0.000001;
    private static final double KI = 0.0000001;//0.00000001
    private static final double KD = 0.0000001;

    private double prevError = 0D;
    private double integral = 0;

    private KalminFilter kalminFilter = new KalminFilter();

    public double getAdjustment(double rpm, double reqEtps, long deltaTime) {

        double filteredRpm = kalminFilter.applyFilter(rpm);

        double error = reqEtps - filteredRpm;

        integral = integral + error * deltaTime;//calculate integral of error
        double derivative = (error - prevError) / deltaTime;//calculate derivative of data

        if (Math.abs(error) < DEAD_BAND) {
            integral = 0;
            derivative = 0;
        }
        prevError = error;
        return KP * error + KD * derivative + KI * integral;
    }


}
