package org.firstinspires.ftc.teamcode.Shashank.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Sam.twoControllerTeleopv2;

/**
 * Created by spmega on 1/5/17.
 */

public class RPMRunnable implements Runnable {
    private Telemetry telemetry = null;
    private ElapsedTime runtime = null;
    private boolean requestStop = false;

    private DcMotor shooter1;
    private DcMotor shooter2;

    public RPMRunnable(DcMotor shooter1, DcMotor shooter2, shooterSettings RPM955, Telemetry telemetry) {
        this.shooter1 = shooter1;
        this.shooter2 = shooter2;
        this.RPM955 = RPM955;
        this.telemetry = telemetry;
    }

    public class shooterSettings{//data members can be replaced, but default values are for 1750 ETPS = 955 RPM

        public shooterSettings(){}
        public shooterSettings(double therequestedRPM, double theoriginalPWR1, double theoriginalPWR2){
            requestedRPM=therequestedRPM;
            requestedEncoderTicksPerSecond =requestedRPM*110/60;
            originalPWR1=theoriginalPWR1;
            originalPWR2=theoriginalPWR2;
            requiredPWR1=originalPWR1;
            requiredPWR2=originalPWR2;
        }

        private double requestedRPM =955;//955;
        private double requestedEncoderTicksPerSecond =requestedRPM*110/60;//1750

        //PID variables
        private double dt=0;
        private double previous_position1=0;
        private double current_position1=0;
        private double current_rpm1=0;
        private double previous_rpm1=0;
        private double error1=0;
        private double previous_error1=0;
        private double integral1=0;
        private double derivative1=0;
        private double adjustment1=0;
        private double previous_position2=0;
        private double current_position2=0;
        private double current_rpm2=0;
        private double previous_rpm2=0;
        private double error2=0;
        private double previous_error2=0;
        private double integral2=0;
        private double derivative2=0;
        private double adjustment2=0;

        //PID Constants
        double Kp = 0.000001;
        double Ki = 0.0000001;//0.00000001
        double Kd = 0.0000001;

        //Timing variables
        public double rampUpTime=1.5;

        //Power Variables
        public double originalPWR1=0.42;
        public double originalPWR2=0.42;
        public final double allowedPowerDifference=0.03;
        public double requiredPWR1=originalPWR1;
        public double requiredPWR2=originalPWR2;
        public double deadband=20;





        //Kalman Filter Variables
        double input1=0;
        double prevXk1=0;
        double prevPk1=1;
        double Xk1=0;
        double Pk1=1;
        double Kk1=0;
        double R1=0.2;

        double input2=0;
        double prevXk2=0;
        double prevPk2=1;
        double Xk2=0;
        double Pk2=1;
        double Kk2=0;
        double R2=0.2;



    }

    private boolean USE_TELEMETRY=true;

    shooterSettings RPM955;

    public double startShootingtime=0;
    public double prevTime=0;

    @Override
    public void run() {
        //if you need to run a loop, make sure that you check that requestStop is still false
        while (!requestStop) {
            //simply printing out and updating the telemetry log here
            int shooting1= shooter1.getCurrentPosition();
            int shooting2= shooter2.getCurrentPosition();

        }
        //if you are not running a loop, make sure to periodically check if the requestStop is still false
        telemetry.log().add("This is ending a thread: " + runtime.toString());
    }

    //call this method when you want to stop the thread
    public void requestStop(){
        requestStop = true;
    }
}
