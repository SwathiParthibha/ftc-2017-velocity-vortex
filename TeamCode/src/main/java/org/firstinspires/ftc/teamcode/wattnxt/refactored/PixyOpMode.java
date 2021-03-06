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
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.ReentrantLock;

@TeleOp(name="PixyOpMode", group="Pixy")
public class PixyOpMode extends OpMode {
    private final double LOADED_POSITION_TRIGGER = 0.557;
    private final double SHOOT_POSITION_TRIGGER = 0.428;
    private final double START_POSITION_ANGULAR = 0.1;
    private final double START_POSITION_TURRET = 0.50;
    private final double CLOCKWISE_POSITION_TURRET = 0.49;
    private final double COUNTERCLOCKWISE_POSITION_TURRET = 0.51;
    private final double SERVO_INCREMENT = 0.0025;
    private int counter = 0;

    private Servo triggerServo, angularServo, turretServo;

    private static final int PORT = 0;
    private static final int ADDRESS = 0x54;
    private static final int DELAY = 10;
    private static final int LOOP_DELAY = 300;
    private static final String DIM_NAME = "Pixy dim";

    private PixyCam pixyCam;
    private ScheduledFuture pixycamscheduledFuture = null;
    private ScheduledFuture updateListscheduledFuture = null;
    private ScheduledExecutorService scheduledExecutorService = Executors.newScheduledThreadPool(2);
    private List<PixyObject> newPixyObjects = new ArrayList<>();

    private ReentrantLock reentrantLock = new ReentrantLock();

    @Override
    public void init() {
        DeviceInterfaceModule dim = hardwareMap.deviceInterfaceModule.get(DIM_NAME);
        pixyCam = new PixyCam(PORT, ADDRESS, dim);

        while (!pixyCam.isPortReady()) {
            telemetry.addData("PixyCam", "Waiting for the port to be ready...");
            telemetry.update();
        }

        pixyCam.init();

        triggerServo = hardwareMap.servo.get("triggerServo");
        angularServo = hardwareMap.servo.get("pixyyaxis");
        turretServo = hardwareMap.servo.get("pixyyrotation");

        triggerServo.setPosition(LOADED_POSITION_TRIGGER);
        angularServo.setPosition(START_POSITION_ANGULAR);
        turretServo.setPosition(START_POSITION_TURRET);

        //running the pixycam thread
        pixycamscheduledFuture = scheduledExecutorService.scheduleAtFixedRate(pixyCam, 0, DELAY, TimeUnit.MILLISECONDS);

        //updating the opmode list every 300 milliseconds, with a second initial delay
        updateListscheduledFuture =  scheduledExecutorService.scheduleAtFixedRate(new Runnable() {
            @Override
            public void run() {
                reentrantLock.lock();
                newPixyObjects.clear();
                int whileCounter = 1;
                while (pixyCam.hasNewObject()) {
                    DbgLog.msg("Amount of times looping through the transfer loop: " + whileCounter);
                    newPixyObjects.add(pixyCam.popObject());
                    whileCounter++;
                }
                reentrantLock.unlock();
            }
        }, 1000, 300, TimeUnit.MILLISECONDS);
    }



    @Override
    public void loop() {
        DbgLog.msg("LOOP START: " + counter);
        if (pixyCam.newObjectCount() == 0) {
            return;
        }

        reentrantLock.lock();
        String pixyObjectsString = newPixyObjects.size() + "\n";
        for (PixyObject pixyObject : newPixyObjects) {
            pixyObjectsString += pixyObject.toString() + "\n";
        }
        reentrantLock.unlock();

        int yAxisDifference = 0;
        int xAxisDifference = 0;

        if(newPixyObjects.size() > 0){
            yAxisDifference = newPixyObjects.get(0).centerY - 120;
            xAxisDifference = newPixyObjects.get(0).centerX - 150;
        }

        if(xAxisDifference > 30){
            DbgLog.msg("UPDATING SERVO POSITION TURRENT");
            for (long i = System.currentTimeMillis(); System.currentTimeMillis() - i > 300;){
                turretServo.setPosition(CLOCKWISE_POSITION_TURRET);
            }
        } else if (xAxisDifference < -30){
            DbgLog.msg("UPDATING SERVO POSITION TURRENT");
            for (long i = System.currentTimeMillis(); System.currentTimeMillis() - i > 300;){
                turretServo.setPosition(COUNTERCLOCKWISE_POSITION_TURRET);
            }
        } else {
            turretServo.setPosition(START_POSITION_TURRET);
        }

        if(yAxisDifference > 30){
            DbgLog.msg("UPDATING SERVO POSITION ANGULAR");
            angularServo.setPosition(angularServo.getPosition() + SERVO_INCREMENT);
            for (long i = System.currentTimeMillis(); System.currentTimeMillis() - i > 300;){
                angularServo.setPosition(angularServo.getPosition() + SERVO_INCREMENT);
            }
        } else if (yAxisDifference < -30){
            DbgLog.msg("UPDATING SERVO POSITION ANGULAR");
            for (long i = System.currentTimeMillis(); System.currentTimeMillis() - i > 300;){
                angularServo.setPosition(angularServo.getPosition() - SERVO_INCREMENT);
            }
        } else {
            angularServo.setPosition(angularServo.getPosition());
        }

        telemetry.addData("Turrent Servo", turretServo.getPosition());
        telemetry.addData("Angular Servo", angularServo.getPosition());
        telemetry.addData("yAxisDifference", yAxisDifference);
        telemetry.addData("xAxisDifference", xAxisDifference);
        telemetry.addData("New PixyObjects", pixyObjectsString);
        telemetry.update();

        DbgLog.msg("LOOP END: " + counter);
        counter++;
    }

    @Override
    public void stop() {
        super.stop();
        pixycamscheduledFuture.cancel(false);
        updateListscheduledFuture.cancel(false);
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
