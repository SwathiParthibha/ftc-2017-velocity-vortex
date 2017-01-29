package org.firstinspires.ftc.teamcode.Sam;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Two Controller Teleop V2", group = "Teleop")
public class twoControllerTeleopv2 extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor scooper;
    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor sweeper;

    private boolean state;
    boolean swap=false;


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
    shooterSettings RPM0;
    shooterSettings RPM800;



    public double startShootingtime=0;
    public double prevTime=0;





    @Override
    public void init() {
        leftMotor = this.hardwareMap.dcMotor.get("l");
        rightMotor = this.hardwareMap.dcMotor.get("r");
        scooper = this.hardwareMap.dcMotor.get("scooper");
        shooter1 = this.hardwareMap.dcMotor.get("shooter1");
        shooter2 = this.hardwareMap.dcMotor.get("shooter2");
        sweeper = this.hardwareMap.dcMotor.get("sweeper");
        state = false;


        RPM955= new shooterSettings();//default settings are for 955, 0.43,0.43
        RPM0 = new shooterSettings(0,0,0);
        RPM800 = new shooterSettings(800,0.35,0.35);


        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        swap=true;

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    @Override
    public void loop() {

        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;
        int shooting1= shooter1.getCurrentPosition();
        int shooting2= shooter2.getCurrentPosition();

        if(swap==true)
        {
            double temp=left;
            left=right;
            right=temp;
        }

        left=scaleInput(left);
        right=scaleInput(right);

        leftMotor.setPower(left);
        rightMotor.setPower(right);

        if(gamepad1.dpad_down){
            leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            swap=false;
        } else if(gamepad1.dpad_up){
            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            swap=true;
        }

        if(gamepad2.dpad_right){
            sweeper.setPower(0.7);
            scooper.setPower(1);
        }

        if(gamepad2.left_trigger > 0){
            scooper.setPower(-0.7);
        } else if(gamepad2.left_bumper){
            scooper.setPower(1);
        } else {
            scooper.setPower(0);
        }

        if(gamepad2.a){
            EncoderShooter(RPM800);
        } else if(gamepad2.b) {
            EncoderShooter(RPM955);//0.6//0.7
            //power=0.7;
            //startrunnning=true;
        }
        else {
            EncoderShooter(RPM0);
        }





            if(gamepad2.right_bumper){
                sweeper.setPower(0.7);
            } else if(gamepad2.right_trigger > 0){
                sweeper.setPower(-0.7);
            } else {
                sweeper.setPower(0);

            }


        telemetry.addData("left joystick",  "%.2f", left);
        telemetry.addData("right joystick", "%.2f", right);
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
    }



    public void EncoderShooter(shooterSettings settings)
    {
        if(settings.requestedRPM!=0) {


            if(startShootingtime==-999) {//only update on first run
                startShootingtime = getRuntime();
            }

            settings.dt=getRuntime()-prevTime;
            if (settings.dt> 0.01) {//only update every 10ms
                settings.current_position1 = shooter1.getCurrentPosition();//MUST BE FIRST - time sensitive measurement
                settings.current_position2 = shooter2.getCurrentPosition();//MUST BE FIRST - time sensitive measurement
                prevTime = getRuntime();//MUST BE FIRST - time sensitive measurement

                updateRPM1and2(settings);

                if(getRuntime()-startShootingtime>settings.rampUpTime) {//only update Kalmin and PID after ramp up
                    timeUpdate(settings);
                    measurementUpdate(settings);


                    //DbgLog.msg("Time: "+getRuntime()+"RPM1: " + current_rpm1+"RPM2: " + current_rpm2);

                    PID1Update(settings);
                    PID2Update(settings);

                    applyAdjustment1(settings);
                    applyAdjustment2(settings);
                }
                clipPower1(settings);
                clipPower2(settings);

                previous1Update(settings);
                previous2Update(settings);


            }

            checkIfReadyToShoot(settings);
            if(USE_TELEMETRY) {
                outputTelemetry(settings);
            }


            shooter1.setPower(settings.requiredPWR1);
            shooter2.setPower(settings.requiredPWR2);


        }
        else
        {
            shooter1.setPower(0);
            shooter2.setPower(0);
            startShootingtime=-999;
            resetKalmin(settings);
            resetPID(settings);
        }

    }


    public void updateRPM1and2(shooterSettings settings){
        settings.current_rpm1 = (settings.current_position1 - settings.previous_position1) / (settings.dt);
        settings.current_rpm2 = (settings.current_position2 - settings.previous_position2) / (settings.dt);
    }

    public void PID1Update(shooterSettings settings){
        settings.error1=-(settings.Xk1- settings.requestedEncoderTicksPerSecond);
        settings.integral1 = settings.integral1 + settings.error1 * settings.dt;//calculate integral of error
        settings.derivative1 = (settings.error1 - settings.previous_error1) / settings.dt;//calculate derivative of data

        if(Math.abs(settings.error1)<settings.deadband)
        {
            settings.integral1=0;
            settings.derivative1=0;
        }

        settings.adjustment1 = settings.Kp * settings.error1 + settings.Kd*settings.derivative1 + settings.Ki*settings.integral1;// + Ki * integral1 + Kd * derivative1;//summation of PID



    }

    public void PID2Update(shooterSettings settings){

        settings.error2=-(settings.Xk2- settings.requestedEncoderTicksPerSecond);
        settings.integral2 = settings.integral2 + settings.error2 * settings.dt;//calculate integral of error
        settings.derivative2 = (settings.error2 - settings.previous_error2) / settings.dt;//calculate derivative of data

        if(Math.abs(settings.error2)<settings.deadband)
        {
            settings.integral2=0;
            settings.derivative2=0;
        }

        settings.adjustment2 = settings.Kp * settings.error2 + settings.Kd*settings.derivative2 + settings.Ki*settings.integral2;// + Ki * integral1 + Kd * derivative1;//summation of PID


    }

    public void previous1Update(shooterSettings settings){
        settings.previous_error1=settings.error1;
        settings.previous_position1 = settings.current_position1;
        settings.previous_rpm1 = settings.current_rpm1;
    }

    public void previous2Update(shooterSettings settings){
        settings.previous_error2=settings.error2;
        settings.previous_position2 = settings.current_position2;
        settings.previous_rpm2 = settings.current_rpm2;
    }

    public void applyAdjustment1(shooterSettings settings) {
        settings.requiredPWR1+=settings.adjustment1;
    }

    public void applyAdjustment2(shooterSettings settings) {
        settings.requiredPWR2+=settings.adjustment2;
    }

    public void clipPower1(shooterSettings settings){
        if(settings.requiredPWR1<settings.originalPWR1-settings.allowedPowerDifference)
        {
            settings.requiredPWR1=settings.originalPWR1-settings.allowedPowerDifference;
        }
        else if(settings.requiredPWR1>settings.originalPWR1+settings.allowedPowerDifference)
        {
            settings.requiredPWR1=settings.originalPWR1+settings.allowedPowerDifference;
        }

    }

    public void clipPower2(shooterSettings settings){
        if(settings.requiredPWR2<settings.originalPWR2-settings.allowedPowerDifference)
        {
            settings.requiredPWR2=settings.originalPWR2-settings.allowedPowerDifference;
        }
        else if(settings.requiredPWR2>settings.originalPWR2+settings.allowedPowerDifference)
        {
            settings.requiredPWR2=settings.originalPWR2+settings.allowedPowerDifference;
        }

    }


    public boolean checkIfReadyToShoot(shooterSettings settings) {
        if(Math.abs(settings.error1)<settings.deadband && Math.abs(settings.error2)<settings.deadband && getRuntime()-startShootingtime>settings.rampUpTime)
        {
            telemetry.addData("READY TO SHOOT", "");
            return true;
        }
        else
        {
            return false;
        }

    }

    public void outputTelemetry(shooterSettings settings) {
        telemetry.addData("requiredPWR1: ", String.format("%.4f", settings.requiredPWR1));
        telemetry.addData("requiredPWR2: ", String.format("%.4f", settings.requiredPWR2));
        telemetry.addData("adjustment1: ", settings.adjustment1);
        telemetry.addData("P1: ", settings.Kp*settings.error1);
        telemetry.addData("I1: ", settings.Ki*settings.integral1);
        telemetry.addData("D1: ", settings.Kd*settings.derivative1);
        telemetry.addData("adjustment2: ", settings.adjustment2);
        telemetry.addData("P2: ", settings.Kp*settings.error2);
        telemetry.addData("I2: ", settings.Ki*settings.integral2);
        telemetry.addData("D2: ", settings.Kd*settings.derivative2);
        telemetry.addData("curr1", settings.current_rpm1);
        telemetry.addData("curr2", settings.current_rpm2);
        telemetry.addData("Kalmin1", settings.Xk1);
        telemetry.addData("Kalmin2", settings.Xk2);
        telemetry.addData("K1", settings.Kk1);
        telemetry.addData("K2", settings.Kk2);
        telemetry.addData("Time: ", "" + getRuntime());
        telemetry.addData("ReqestedETPS", settings.requestedEncoderTicksPerSecond);

    }

    //Kalmin phase 1
    public void timeUpdate(shooterSettings settings){
        settings.input1=settings.current_rpm1;
        settings.prevXk1=settings.Xk1;
        settings.prevPk1=settings.Pk1;

        settings.input2=settings.current_rpm2;
        settings.prevXk2=settings.Xk2;
        settings.prevPk2=settings.Pk2;
    }

    //Kalmin phase 2
    public void measurementUpdate(shooterSettings settings){
        //RPM1 calculations
        settings.Kk1=settings.prevPk1/(settings.prevPk1+settings.R1);
        settings.Xk1=settings.prevXk1+settings.Kk1*(settings.input1-settings.prevXk1);
        settings.Pk1=(1-settings.Kk1)*settings.prevPk1;

        //RPM2 calculations
        settings.Kk2=settings.prevPk2/(settings.prevPk2+settings.R2);
        settings.Xk2=settings.prevXk2+settings.Kk2*(settings.input2-settings.prevXk2);
        settings.Pk2=(1-settings.Kk2)*settings.prevPk2;


    }

    public void resetKalmin(shooterSettings settings){
        settings.input1=0;
        settings.prevXk1=0;
        settings.prevPk1=1;
        settings.Xk1=0;
        settings.Pk1=1;
        // Kk1=0;


        settings.input2=0;
        settings.prevXk2=0;
        settings.prevPk2=1;
        settings.Xk2=0;
        settings.Pk2=1;
        // Kk2=0;



    }

    public void resetPID(shooterSettings settings){
        settings.previous_position1=0;
        settings.current_position1=0;
        settings.current_rpm1=0;
        settings.previous_rpm1=0;
        settings.error1=0;
        settings.previous_error1=0;
        settings.integral1=0;
        settings.derivative1=0;
        settings.adjustment1=0;
        settings.previous_position2=0;
        settings.current_position2=0;
        settings.current_rpm2=0;
        settings.previous_rpm2=0;
        settings.error2=0;
        settings.previous_error2=0;
        settings.integral2=0;
        settings.derivative2=0;
        settings.adjustment2=0;


    }

















    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }


}



