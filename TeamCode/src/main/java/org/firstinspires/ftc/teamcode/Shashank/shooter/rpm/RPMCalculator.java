package org.firstinspires.ftc.teamcode.Shashank.shooter.rpm;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by spmeg on 3/21/2017.
 */

public interface RPMCalculator {
    public void getRPM();
    public void addMotor(String key, DcMotor dcMotor);
    public void removeMotors(String key);

}
