package org.firstinspires.ftc.teamcode.wattnxt.refactored;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

@TeleOp(name="PixyOpMode", group="Pixy")
public class PixyOpMode extends OpMode {
    private final double LOADED_POSITION_TRIGGER = 0.557;
    private final double SHOOT_POSITION_TRIGGER = 0.428;
    private final double START_POSITION_ANGULAR = 0.1;
    private final double START_POSITION_TURRET = 0.50;
    private final double CLOCKWISE_POSITION_TURRET = 0.49;
    private final double COUNTERCLOCKWISE_POSITION_TURRET = 0.51;
    private final double SERVO_INCREMENT = 0.0025;

    private Servo triggerServo, angularServo, turretServo;

    private static final int PORT = 0;
    private static final int ADDRESS = 0x54;
    private static final int DELAY = 10;
    private static final String DIM_NAME = "Pixy dim";

    private PixyCam pixyCam;
    private ScheduledFuture scheduledFuture = null;
    private ScheduledExecutorService scheduledExecutorService = Executors.newSingleThreadScheduledExecutor();

    @Override
    public void init() {
        DeviceInterfaceModule dim = hardwareMap.deviceInterfaceModule.get(DIM_NAME);
        pixyCam = new PixyCam(PORT, ADDRESS, dim);

        while (!pixyCam.isPortReady()) {
            telemetry.addData("PixyCam", "Waiting for the port to be ready...");
            telemetry.update();
        }

        triggerServo = hardwareMap.servo.get("triggerServo");
        angularServo = hardwareMap.servo.get("pixyyaxis");
        turretServo = hardwareMap.servo.get("pixyyrotation");

        triggerServo.setPosition(LOADED_POSITION_TRIGGER);
        angularServo.setPosition(START_POSITION_ANGULAR);
        turretServo.setPosition(START_POSITION_TURRET);

        scheduledFuture = scheduledExecutorService.scheduleAtFixedRate(pixyCam, 0, DELAY, TimeUnit.MILLISECONDS);;
    }

    @Override
    public void loop() {
        if (pixyCam.newObjectCount() == 0) {
            return;
        }
        List<PixyObject> newPixyObjects = new ArrayList<>();
        while (pixyCam.hasNewObject()) {
            newPixyObjects.add(pixyCam.popObject());
        }
        String pixyObjectsString = newPixyObjects.size() + "\n";
        for (PixyObject pixyObject : newPixyObjects) {
            pixyObjectsString += pixyObject.toString() + "\n";
        }

        int yAxisDifference = newPixyObjects.get(0).centerY - 90;
        int xAxisDifference = newPixyObjects.get(0).centerX - 150;

        if(xAxisDifference > 20){
            turretServo.setPosition(CLOCKWISE_POSITION_TURRET);
        } else if (xAxisDifference < -20){
            turretServo.setPosition(COUNTERCLOCKWISE_POSITION_TURRET);
        } else {
            turretServo.setPosition(START_POSITION_TURRET);
        }

        if(yAxisDifference > 10){
            angularServo.setPosition(angularServo.getPosition() + SERVO_INCREMENT);
        } else if (yAxisDifference < -10){
            angularServo.setPosition(angularServo.getPosition() - SERVO_INCREMENT);
        } else {
            angularServo.setPosition(angularServo.getPosition());
        }

        telemetry.addData("Turrent Servo", turretServo.getPosition());
        telemetry.addData("Angular Servo", angularServo.getPosition());
        telemetry.addData("yAxisDifference", yAxisDifference);
        telemetry.addData("xAxisDifference", xAxisDifference);
        telemetry.addData("New PixyObjects", pixyObjectsString);
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
        scheduledFuture.cancel(true);
        scheduledExecutorService.shutdown();

        try {
            scheduledExecutorService.awaitTermination(1000, TimeUnit.MILLISECONDS);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        scheduledExecutorService.shutdownNow();
        DbgLog.msg("PIXY OPMODE STOPPED");
    }
}
