package org.firstinspires.ftc.teamcode.Sam;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Two Controller Teleop", group = "Teleop")
public class twoControllerTeleop extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor scooper;
    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor sweeper;

    private double RequestedRPM=1750;
    private double power=0;
    private long dt=1000;
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



    private boolean startrunnning=false;
    private boolean running=false;
    private String output="";

    double Kp=0.1;
    double Ki=0.00001;
    double Kd=0.00001;

    private boolean ShooterPowerCont=true;

 /*   final Runnable ShooterPower = new Runnable() {
        public void run() {

            while (ShooterPowerCont) {
                synchronized (this) {
                    try {

                        if(running) {
                            current_position=shooter1.getCurrentPosition();

                            current_rpm = (previous_position - current_position) / (int) dt;

                            error1 = current_rpm - RequestedRPM;

                            integral1 = integral1 + error1 * (int) dt;//calculate integral of error
                            derivative1 = (error1 - previous_error1) / (int) dt;//calculate derivative of data
                            adjustment1 = Kp * error1 + Ki * integral1 + Kd * derivative1;//summation of PID

                            previous_rpm = current_rpm;
                            previous_error = error;
                            previous_position=current_position;

                        }
                        current_position1=shooter1.getCurrentPosition();

                        current_rpm1 = (current_position1 - previous_position1) / (int) dt;

                        adjustment1=Kp*(RequestedRPM-current_rpm1);
                        previous_position1=current_position1;
                        previous_rpm1=current_rpm1;

                        output="error1: "+error1;
                        output+="adjust: "+adjustment1;
                        output+="curr"+current_rpm1;
                        //output+="power: "+power;


                        current_position2=shooter2.getCurrentPosition();

                        current_rpm2 = (current_position2 - previous_position2) / (int) dt;

                        adjustment2=Kp*(RequestedRPM-current_rpm2);
                        previous_position2=current_position2;
                        previous_rpm2=current_rpm2;

                        output="error2: "+error2;
                        output+="adjust: "+adjustment2;
                        output+="curr"+current_rpm2;


                        if(startrunnning)
                        {
                            startrunnning=false;
                            running=true;
                            shooter1.setPower(power);
                            shooter2.setPower(power);
                        }

                        if(running)
                        {
                            shooter1.setPower(power-adjustment1);
                            shooter2.setPower(power-adjustment2);
                            //shooter2.setPower(power);
                        }
                        else
                        {
                            integral1=0;
                            previous_rpm1=0;
                            previous_error1=0;
                            previous_position1=0;
                            integral2=0;
                            previous_rpm2=0;
                            previous_error2=0;
                            previous_position2=0;
                            shooter1.setPower(0);
                            shooter2.setPower(0);
                            //shooter2.setPower(power);
                        }



                        sleep(dt);
                    } catch (Exception e) {
                    }
                }
            }
        }
    };
    final Thread Shooter = new Thread(ShooterPower);
*/

    private boolean state;
    boolean swap=false;



    @Override
    public void init() {
        leftMotor = this.hardwareMap.dcMotor.get("l");
        rightMotor = this.hardwareMap.dcMotor.get("r");
        scooper = this.hardwareMap.dcMotor.get("scooper");
        shooter1 = this.hardwareMap.dcMotor.get("shooter1");
        shooter2 = this.hardwareMap.dcMotor.get("shooter2");
        sweeper = this.hardwareMap.dcMotor.get("sweeper");
        state = false;

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

//        Shooter.start();

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
            EncoderShooter(0.55);//0.7//0.9
        } else if(gamepad2.b) {
            EncoderShooter(0.8);//0.6//0.7
            //power=0.7;
            //startrunnning=true;
        }
        else if(gamepad2.y)
        {
            //EncoderShooter(0.2);
        }
        else {
            EncoderShooter(0);
            power=0;
            running=false;
        }


        if(gamepad2.left_stick_y>0.3)
        {
            RequestedRPM+=5;
        }
        else if(gamepad2.left_stick_y<-0.3)
        {
            RequestedRPM-=5;
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
        telemetry.addData("shooting1", shooting1);
        telemetry.addData("shooting2", shooting2);
        telemetry.addData("RequestedPWM", RequestedRPM);
        //telemetry.addData("Out",output);
        telemetry.update();
    }

    @Override
    public void stop() {
        ShooterPowerCont=false;
        super.stop();
    }


    public double prevTime=0;

    public double requiredPWR1=0.43;
    public double requiredPWR2=0.43;
    public double runningAvgRPM1=0;
    public double runningAvgRPM2=0;
    public double prevResetTime=0;

    public double count=1;


    public void updateAvgRPM(double speed)
    {
       // if(getRuntime()-prevResetTime>3.0)
       // {
       //     count=1;
       // }



        if(speed==0)
        {
            runningAvgRPM1=1;
            runningAvgRPM2=1;
            count=1;

            shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            return;
        }
        else {

            if (getRuntime() - prevTime > 0.1) {//only update every 10ms
                current_position1 = shooter1.getCurrentPosition();
                current_position2 = shooter2.getCurrentPosition();
                current_rpm1 = (current_position1 - previous_position1) / (getRuntime() - prevTime);
                current_rpm2 = (current_position2 - previous_position2) / (getRuntime() - prevTime);

                previous_position1 = current_position1;
                previous_rpm1 = current_rpm1;
                previous_position2 = current_position2;
                previous_rpm2 = current_rpm2;
                prevTime = getRuntime();




                /*runningAvgRPM1=runningAvgRPM1/(count+1);
                runningAvgRPM1*=count;
                runningAvgRPM1*=current_rpm1/(count+1);

                runningAvgRPM2=runningAvgRPM2/(count+1);
                runningAvgRPM2*=count;
                runningAvgRPM2*=current_rpm2/(count+1);
                */

                runningAvgRPM1*=count;
                runningAvgRPM1+=current_rpm1;
                runningAvgRPM2*=count;
                runningAvgRPM2+=current_rpm2;
                count++;
                runningAvgRPM1/=count;
                runningAvgRPM2/=count;

            }
        }
    }


/*    public void EncoderShooter(double speed)
    {
        updateAvgRPM(speed);
        if(speed!=0) {

            double Kp = 0.01;
            double Ki = 0.00001;
            double Kd = 0.00001;




                if(runningAvgRPM1<RequestedRPM)
                {//we need to speed up
                    requiredPWR1+=Kp;
                }
                else if(runningAvgRPM1>RequestedRPM)
                {//we need to slow down
                    requiredPWR1-=Kp;
                }
                if(runningAvgRPM2<RequestedRPM)
                {//we need to speed up
                    requiredPWR2+=Kp;
                }
                else if(runningAvgRPM2>RequestedRPM)
                {//we need to slow down
                    requiredPWR2-=Kp;
                }




            telemetry.addData("requiredPWR1: ", String.format("%.4f", requiredPWR1));
            telemetry.addData("requiredPWR2: ", String.format("%.4f", requiredPWR2));
            telemetry.addData("curr1", runningAvgRPM2);
            telemetry.addData("curr2", runningAvgRPM2);
            telemetry.addData("Time: ", "" + getRuntime());



            shooter1.setPower(requiredPWR1);
            shooter2.setPower(requiredPWR2);
            //shooter2.setPower(speed);

        }
        else
        {
            shooter1.setPower(0);
            shooter2.setPower(0);
        }

    }
    */
int currcount=0;



    public void EncoderShooter(double speed)
    {
        if(speed!=0) {

            double Kp = 0.001;
            double Ki = 0.00001;
            double Kd = 0.00001;


            double timeDiff=getRuntime()-prevTime;
            if (timeDiff> 0.01) {//only update every 10ms
                current_position1 = shooter1.getCurrentPosition();
                current_position2 = shooter2.getCurrentPosition();
                prevTime = getRuntime();
                current_rpm1 = (current_position1 - previous_position1) / (timeDiff);
                current_rpm2 = (current_position2 - previous_position2) / (timeDiff);



                if(currcount>20) {
                    timeUpdate(current_rpm1, current_rpm2);
                    measurementUpdate();
                }else
                {
                    currcount++;
                }

                DbgLog.msg("Time: "+getRuntime()+"RPM1: " + current_rpm1+"RPM2: " + current_rpm2);



                if (current_rpm1 < RequestedRPM) {//we need to speed up
                    requiredPWR1 += Kp;
                } else if (current_rpm1 > RequestedRPM) {//we need to slow down
                    requiredPWR1 -= Kp;
                }



                if (current_rpm2 < RequestedRPM) {//we need to speed up
                    requiredPWR2 += Kp;
                } else if (current_rpm2 > RequestedRPM) {//we need to slow down
                    requiredPWR2 -= Kp;
                }

                previous_position1 = current_position1;
                previous_rpm1 = current_rpm1;
                previous_position2 = current_position2;
                previous_rpm2 = current_rpm2;

            }


            telemetry.addData("requiredPWR1: ", String.format("%.4f", requiredPWR1));
            telemetry.addData("requiredPWR2: ", String.format("%.4f", requiredPWR2));
            telemetry.addData("curr1", current_rpm1);
            telemetry.addData("curr2", current_rpm2);
            telemetry.addData("Kalmin1", Xk1);
            telemetry.addData("Kalmin2", Xk2);
            telemetry.addData("Time: ", "" + getRuntime());



            shooter1.setPower(requiredPWR1);
            shooter2.setPower(requiredPWR2);
            //shooter2.setPower(speed);

        }
        else
        {
            shooter1.setPower(0);
            shooter2.setPower(0);
            resetKalmin();
        }

    }

    public boolean isClose(double x, double y, double distance)
    {
        if(Math.abs(x-y)<distance)
        {
            return true;
        }
        else{
            return false;
        }

    }


    public double scaleShooterPower(double intialPower)
    {
        double MAX_VOLTAGE=13.7;

        double currentVoltage= hardwareMap.voltageSensor.get("drive").getVoltage();

        double scaledPower=MAX_VOLTAGE*intialPower/currentVoltage;

        telemetry.addData("Scaled power: ", scaledPower);

        return scaledPower;



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

    public void timeUpdate(double curr1, double curr2)
    {
        //cout<<"Enter input: ";
        //cin>>input;

        input1=curr1;
        prevXk1=Xk1;
        prevPk1=Pk1;

        input2=curr2;
        prevXk2=Xk2;
        prevPk2=Pk2;
    }

    public void measurementUpdate()
    {
        //RPM1 calculations
        Kk1=prevPk1/(prevPk1+R1);
        Xk1=prevXk1+Kk1*(input1-prevXk1);
        Pk1=(1-Kk1)*prevPk1;

        //RPM2 calculations
        Kk2=prevPk2/(prevPk2+R2);
        Xk2=prevXk2+Kk2*(input2-prevXk2);
        Pk2=(1-Kk2)*prevPk2;


    }

    public void resetKalmin()
    {
        input1=0;
        prevXk1=0;
        prevPk1=1;
        Xk1=0;
        Pk1=1;
        Kk1=0;


        input2=0;
        prevXk2=0;
        prevPk2=1;
        Xk2=0;
        Pk2=1;
        Kk2=0;



    }


}



