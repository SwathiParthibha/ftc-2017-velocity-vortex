package org.firstinspires.ftc.teamcode.Saransh;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static java.lang.Thread.sleep;


@TeleOp(name = "Negative Teleop", group = "Teleop")
public class EncoderTeleopNegative extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor scooper;
    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor sweeper;

    private double RequestedRPM=1.9;
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

    private boolean startrunnning=false;
    private boolean running=false;
    private String output="";

    double Kp=0.1;
    double Ki=0.00001;
    double Kd=0.00001;

    private boolean ShooterPowerCont=true;

    final Runnable ShooterPower = new Runnable() {
        public void run() {

            while (ShooterPowerCont) {
                synchronized (this) {
                    try {

                       /* if(running) {
                            current_position=shooter1.getCurrentPosition();

                            current_rpm = (previous_position - current_position) / (int) dt;

                            error = current_rpm - RequestedRPM;
                            //abserror=(int)Math.abs(current_rpm-RequestedRPM);
                            integral = integral + error * (int) dt;//calculate integral of error
                            derivative = (error - previous_error) / (int) dt;//calculator derivative of data
                            adjustment = Kp * error + Ki * integral + Kd * derivative;//summation of PID

                            previous_rpm = current_rpm;
                            previous_error = error;
                            previous_position=current_position;

                        }*/
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

        Shooter.start();

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
            EncoderShooter(scaleShooterPower(0.55));//0.7//0.9
        } else if(gamepad2.b) {
            //EncoderShooter(scaleShooterPower(0.8));//0.6//0.7
            power=0.7;
            startrunnning=true;
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
        telemetry.addData("Out",output);
        telemetry.update();
    }

    @Override
    public void stop() {
        ShooterPowerCont=false;
        super.stop();
    }


    public void EncoderShooter(double speed)
    {

        shooter1.setPower(speed);
        shooter2.setPower(speed);


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

}



