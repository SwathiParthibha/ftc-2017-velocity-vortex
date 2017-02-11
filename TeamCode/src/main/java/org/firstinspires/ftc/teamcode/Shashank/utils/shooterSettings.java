package org.firstinspires.ftc.teamcode.Shashank.utils;

public class shooterSettings {//data members can be replaced, but default values are for 1750 ETPS = 955 RPM

        public shooterSettings(){}
        public shooterSettings(double therequestedRPM, double theoriginalPWR1, double theoriginalPWR2){
            requestedRPM=therequestedRPM;
            requestedEncoderTicksPerSecond =requestedRPM*110/60;
            originalPWR1=theoriginalPWR1;
            originalPWR2=theoriginalPWR2;
            requiredPWR1=originalPWR1;
            requiredPWR2=originalPWR2;
        }

        public double requestedRPM =955;//955;
        public double requestedEncoderTicksPerSecond =requestedRPM*110/60;//1750

        //PID variables
        public double dt=0;
        public double previous_position1=0;
        public double current_position1=0;
        public double current_rpm1=0;
        public double previous_rpm1=0;
        public double error1=0;
        public double previous_error1=0;
        public double integral1=0;
        public double derivative1=0;
        public double adjustment1=0;
        public double previous_position2=0;
        public double current_position2=0;
        public double current_rpm2=0;
        public double previous_rpm2=0;
        public double error2=0;
        public double previous_error2=0;
        public double integral2=0;
        public double derivative2=0;
        public double adjustment2=0;

        //PID Constants
        public double Kp = 0.00000001;
        public double Ki = 0.0000000000000001;//0.00000001
        public double Kd = 0.0000001;

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
        public double input1=0;
        public double prevXk1=0;
        public double prevPk1=1;
        public double Xk1=0;
        public double Pk1=1;
        public double Kk1=0;
        public double R1=0.2;

        public double input2=0;
        public double prevXk2=0;
        public double prevPk2=1;
        public double Xk2=0;
        public double Pk2=1;
        public double Kk2=0;
        public double R2=0.2;

        public double getRequestedRPM() {
                return requestedRPM;
        }

        public void setRequestedRPM(double requestedRPM) {
                this.requestedRPM = requestedRPM;
        }

        public double getRequestedEncoderTicksPerSecond() {
                return requestedEncoderTicksPerSecond;
        }

        public void setRequestedEncoderTicksPerSecond(double requestedEncoderTicksPerSecond) {
                this.requestedEncoderTicksPerSecond = requestedEncoderTicksPerSecond;
        }

        public double getDt() {
                return dt;
        }

        public void setDt(double dt) {
                this.dt = dt;
        }

        public double getPrevious_position1() {
                return previous_position1;
        }

        public void setPrevious_position1(double previous_position1) {
                this.previous_position1 = previous_position1;
        }

        public double getCurrent_position1() {
                return current_position1;
        }

        public void setCurrent_position1(double current_position1) {
                this.current_position1 = current_position1;
        }

        public double getCurrent_rpm1() {
                return current_rpm1;
        }

        public void setCurrent_rpm1(double current_rpm1) {
                this.current_rpm1 = current_rpm1;
        }

        public double getPrevious_rpm1() {
                return previous_rpm1;
        }

        public void setPrevious_rpm1(double previous_rpm1) {
                this.previous_rpm1 = previous_rpm1;
        }

        public double getError1() {
                return error1;
        }

        public void setError1(double error1) {
                this.error1 = error1;
        }

        public double getPrevious_error1() {
                return previous_error1;
        }

        public void setPrevious_error1(double previous_error1) {
                this.previous_error1 = previous_error1;
        }

        public double getIntegral1() {
                return integral1;
        }

        public void setIntegral1(double integral1) {
                this.integral1 = integral1;
        }

        public double getDerivative1() {
                return derivative1;
        }

        public void setDerivative1(double derivative1) {
                this.derivative1 = derivative1;
        }

        public double getAdjustment1() {
                return adjustment1;
        }

        public void setAdjustment1(double adjustment1) {
                this.adjustment1 = adjustment1;
        }

        public double getPrevious_position2() {
                return previous_position2;
        }

        public void setPrevious_position2(double previous_position2) {
                this.previous_position2 = previous_position2;
        }

        public double getCurrent_position2() {
                return current_position2;
        }

        public void setCurrent_position2(double current_position2) {
                this.current_position2 = current_position2;
        }

        public double getCurrent_rpm2() {
                return current_rpm2;
        }

        public void setCurrent_rpm2(double current_rpm2) {
                this.current_rpm2 = current_rpm2;
        }

        public double getPrevious_rpm2() {
                return previous_rpm2;
        }

        public void setPrevious_rpm2(double previous_rpm2) {
                this.previous_rpm2 = previous_rpm2;
        }

        public double getError2() {
                return error2;
        }

        public void setError2(double error2) {
                this.error2 = error2;
        }

        public double getPrevious_error2() {
                return previous_error2;
        }

        public void setPrevious_error2(double previous_error2) {
                this.previous_error2 = previous_error2;
        }

        public double getIntegral2() {
                return integral2;
        }

        public void setIntegral2(double integral2) {
                this.integral2 = integral2;
        }

        public double getDerivative2() {
                return derivative2;
        }

        public void setDerivative2(double derivative2) {
                this.derivative2 = derivative2;
        }

        public double getAdjustment2() {
                return adjustment2;
        }

        public void setAdjustment2(double adjustment2) {
                this.adjustment2 = adjustment2;
        }

        public double getKp() {
                return Kp;
        }

        public void setKp(double kp) {
                Kp = kp;
        }

        public double getKi() {
                return Ki;
        }

        public void setKi(double ki) {
                Ki = ki;
        }

        public double getKd() {
                return Kd;
        }

        public void setKd(double kd) {
                Kd = kd;
        }

        public double getRampUpTime() {
                return rampUpTime;
        }

        public void setRampUpTime(double rampUpTime) {
                this.rampUpTime = rampUpTime;
        }

        public double getOriginalPWR1() {
                return originalPWR1;
        }

        public void setOriginalPWR1(double originalPWR1) {
                this.originalPWR1 = originalPWR1;
        }

        public double getOriginalPWR2() {
                return originalPWR2;
        }

        public void setOriginalPWR2(double originalPWR2) {
                this.originalPWR2 = originalPWR2;
        }

        public double getAllowedPowerDifference() {
                return allowedPowerDifference;
        }

        public double getRequiredPWR1() {
                return requiredPWR1;
        }

        public void setRequiredPWR1(double requiredPWR1) {
                this.requiredPWR1 = requiredPWR1;
        }

        public double getRequiredPWR2() {
                return requiredPWR2;
        }

        public void setRequiredPWR2(double requiredPWR2) {
                this.requiredPWR2 = requiredPWR2;
        }

        public double getDeadband() {
                return deadband;
        }

        public void setDeadband(double deadband) {
                this.deadband = deadband;
        }

        public double getInput1() {
                return input1;
        }

        public void setInput1(double input1) {
                this.input1 = input1;
        }

        public double getPrevXk1() {
                return prevXk1;
        }

        public void setPrevXk1(double prevXk1) {
                this.prevXk1 = prevXk1;
        }

        public double getPrevPk1() {
                return prevPk1;
        }

        public void setPrevPk1(double prevPk1) {
                this.prevPk1 = prevPk1;
        }

        public double getXk1() {
                return Xk1;
        }

        public void setXk1(double xk1) {
                Xk1 = xk1;
        }

        public double getPk1() {
                return Pk1;
        }

        public void setPk1(double pk1) {
                Pk1 = pk1;
        }

        public double getKk1() {
                return Kk1;
        }

        public void setKk1(double kk1) {
                Kk1 = kk1;
        }

        public double getR1() {
                return R1;
        }

        public void setR1(double r1) {
                R1 = r1;
        }

        public double getInput2() {
                return input2;
        }

        public void setInput2(double input2) {
                this.input2 = input2;
        }

        public double getPrevXk2() {
                return prevXk2;
        }

        public void setPrevXk2(double prevXk2) {
                this.prevXk2 = prevXk2;
        }

        public double getPrevPk2() {
                return prevPk2;
        }

        public void setPrevPk2(double prevPk2) {
                this.prevPk2 = prevPk2;
        }

        public double getXk2() {
                return Xk2;
        }

        public void setXk2(double xk2) {
                Xk2 = xk2;
        }

        public double getPk2() {
                return Pk2;
        }

        public void setPk2(double pk2) {
                Pk2 = pk2;
        }

        public double getKk2() {
                return Kk2;
        }

        public void setKk2(double kk2) {
                Kk2 = kk2;
        }

        public double getR2() {
                return R2;
        }

        public void setR2(double r2) {
                R2 = r2;
        }
}
