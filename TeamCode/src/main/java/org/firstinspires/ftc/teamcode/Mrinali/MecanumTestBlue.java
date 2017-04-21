/**
 * Created by Mrinali on 3/24/2017.
 */

package org.firstinspires.ftc.teamcode.Mrinali;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Shashank.statemachine.AllianceColor;

import ftclib.FtcOpMode;

@Autonomous(name = "Mecanum Test Blue", group = "Mecanum")
public class MecanumTestBlue extends FtcOpMode {

    MecanumAuto auto = new MecanumAuto(this);

    @Override
    public void initRobot() {

    }

    @Override
    public void runOpMode() throws InterruptedException{

        auto.init(hardwareMap, telemetry, AllianceColor.BLUE);

        while (!isStarted()) {
            telemetry.addLine("Ready to Run");
            telemetry.addData("ODS Light", auto.odsLight.getLightDetected());
            telemetry.addData("ODS Light Raw", auto.odsLight.getRawLightDetected());
            telemetry.addData("Bottom Color Sensor", auto.bottomColorSensor.alpha());
            telemetry.addData("Bottom Color Sensor", auto.bottomColorSensor.argb());
            telemetry.addData("Angle", auto.IMUheading());
            telemetry.addData("Distance Right", auto.getcmUltrasonic(auto.rangeSensorRight));
            telemetry.addData("Distance Left", auto.getcmUltrasonic(auto.rangeSensorLeft));
            telemetry.addData("Left red", auto.leftColorSensor.red());
            telemetry.addData("Right red", auto.rightColorSensor.red());
            telemetry.addData("Front left", auto.frontLeftMotor.getPosition());
            telemetry.addData("Front right", auto.frontRightMotor.getPosition());
            telemetry.addData("Back left", auto.backLeftMotor.getPosition());
            telemetry.addData("Back right", auto.backRightMotor.getPosition());
            telemetry.update();
        }

        auto.toWall();
        auto.toWhiteLine();
        auto.pushButton();
        auto.backup();
        auto.turn(0);
        auto.toWhiteLine();
        auto.pushButton();
        auto.driveBase.mecanumDrive_Polar(.5, 20, 0);
        while (opModeIsActive())
            idle();
    }

    public double IMUheading360() {
        return (auto.imu.getAngularOrientation().firstAngle + 360) %360;
    }
}
