package org.firstinspires.ftc.teamcode.Sam;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Sam.shooter.MotorFactory;
import org.firstinspires.ftc.teamcode.Sam.shooter.RPMThreadMilliseconds;
import org.firstinspires.ftc.teamcode.Sam.shooter.beans.ShooterMotor;
import org.firstinspires.ftc.teamcode.Sam.shooter.power.PowerManager;
import org.firstinspires.ftc.teamcode.Sam.shooter.util.Constants;
import org.firstinspires.ftc.teamcode.Sam.util.Util;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@Disabled
@TeleOp(name = "Encoder Multithreading", group = "Teleop")
public class encoderMultiThreading extends OpMode {
    private DcMotor shooter1;
    private DcMotor shooter2;
    private PowerManager leftShooterPowerMgr;
    private PowerManager rightShooterPowerMgr;


    private ElapsedTime runtime = new ElapsedTime();


    private ScheduledExecutorService scheduledThreadPool = Executors.newScheduledThreadPool(2);

    @Override
    public void init() {

        shooter1 = this.hardwareMap.dcMotor.get("shooter1");
        shooter2 = this.hardwareMap.dcMotor.get("shooter2");

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorFactory motorFactory = MotorFactory.getInstance();
        ShooterMotor leftShooter = new ShooterMotor();
        leftShooter.setName(Constants.MOTORNAME.LEFT_SHOOTER);
        motorFactory.addMotor(leftShooter);

        ShooterMotor rightShooter = new ShooterMotor();
        rightShooter.setName(Constants.MOTORNAME.RIGHT_SHOOTER);
        motorFactory.addMotor(rightShooter);

        leftShooterPowerMgr = new PowerManager(Constants.MOTORNAME.LEFT_SHOOTER, shooter1);
        rightShooterPowerMgr = new PowerManager(Constants.MOTORNAME.RIGHT_SHOOTER, shooter2);

        scheduledThreadPool.scheduleAtFixedRate(new RPMThreadMilliseconds(shooter1, Constants.MOTORNAME.LEFT_SHOOTER), 0L, Constants.DELTA_TIME, TimeUnit.MILLISECONDS);
        scheduledThreadPool.scheduleAtFixedRate(new RPMThreadMilliseconds(shooter2, Constants.MOTORNAME.RIGHT_SHOOTER), 0L, Constants.DELTA_TIME, TimeUnit.MILLISECONDS);
    }


    @Override
    public void start() {
        runtime.reset();
        super.start();
    }

    @Override
    public void loop() {

        leftShooterPowerMgr.regulatePower();
        rightShooterPowerMgr.regulatePower();

        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();

        scheduledThreadPool.shutdown();
        Util.waitUntil(5);
        scheduledThreadPool.shutdownNow();
    }


}