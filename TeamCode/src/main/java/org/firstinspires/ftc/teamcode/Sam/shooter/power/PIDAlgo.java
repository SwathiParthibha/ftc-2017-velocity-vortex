package org.firstinspires.ftc.teamcode.Sam.shooter.power;


public class PIDAlgo {
    private static final double DEAD_BAND = 20;
    private static final double KP = 0.000001;
    private static final double KI = 0.0000001;//0.00000001
    private static final double KD = 0.0000001;
    private double proportionAdjust;
    private double filteredRPM;
    private double integralAdjust;
    private double derivativeAdjust;
    private double prevError = 0D;
    private double integral = 0;

    private KalminFilter kalminFilter = new KalminFilter();


    public double getProportionAdjust() {
        return proportionAdjust;
    }

    public double getIntegralAdjust() {
        return integralAdjust;
    }

    public double getDerivativeAdjust() {
        return derivativeAdjust;
    }

    public double getKalminTrustVal() {
        return kalminFilter.getTrustVal();
    }

    public double getKalminPrevError() {
        return kalminFilter.getPrevError();
    }

    public double getKalminFilteredData() {
        return filteredRPM;
    }

    public double getAdjustment(double rpm, double reqEtps, long deltaTime) {

        filteredRPM = kalminFilter.applyFilter(rpm);

        double error = reqEtps - filteredRPM;

        integral = integral + error * deltaTime;//calculate integral of error
        double derivative = (error - prevError) / deltaTime;//calculate derivative of data

        if (Math.abs(error) < DEAD_BAND) {
            integral = 0;
            derivative = 0;
        }
        prevError = error;
        proportionAdjust = KP * error;
        integralAdjust = KI * integral;
        derivativeAdjust = KD * derivative;

        return proportionAdjust + integralAdjust + derivativeAdjust;
    }

    public void reset() {
        kalminFilter.reset();
        prevError = 0D;
        integral = 0;

    }


}

