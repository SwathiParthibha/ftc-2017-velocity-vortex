/**
 * Created by Mrinali on 3/24/2017.
 */

package org.firstinspires.ftc.teamcode.Mrinali;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Shashank.statemachine.AllianceColor;

import ftclib.FtcOpMode;

@Autonomous(name = "Mecanum Turn Test", group = "Mecanum")
public class MecanumTest2 extends FtcOpMode {

    MecanumAuto auto = new MecanumAuto(this);

    @Override
    public void initRobot() {

    }

    @Override
    public void runOpMode() throws InterruptedException{

        auto.init(hardwareMap, telemetry, AllianceColor.RED);

        while (!isStarted()) {
            telemetry.addLine("Ready to Run");
            telemetry.addData("ODS Light", auto.odsLight.getLightDetected());
            telemetry.addData("ODS Light Raw", auto.odsLight.getRawLightDetected());
            telemetry.addData("Angle", auto.IMUheading());
            telemetry.addData("Distance", auto.getcmUltrasonic(auto.rangeSensorRight));
            telemetry.addData("Front left", auto.frontLeftMotor.getPosition());
            telemetry.addData("Front right", auto.frontRightMotor.getPosition());
            telemetry.addData("Back left", auto.backLeftMotor.getPosition());
            telemetry.addData("Back right", auto.backRightMotor.getPosition());
            telemetry.update();
        }

        auto.turn(90);
        sleep(500);
        telemetry.log().add("Angle " + auto.IMUheading());
        while (opModeIsActive());
    }

    public double IMUheading360() {
        return (auto.imu.getAngularOrientation().firstAngle + 360) %360;
    }
}
