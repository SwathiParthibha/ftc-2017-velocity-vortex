package org.firstinspires.ftc.teamcode.Sam.shooter.power;


public class KalminFilter {

    private static final double STD_DEVIATION=0.2;
    
    private double prevValue=0D;
    private double prevError = 1D;

    public double applyFilter(double rpm){
        double trustVal = prevError / (prevError + STD_DEVIATION);
        double filteredData = prevValue + trustVal * (rpm - prevValue);

        prevError = (1 - trustVal) * prevError;
        prevValue= filteredData;

        return filteredData;
    }

}