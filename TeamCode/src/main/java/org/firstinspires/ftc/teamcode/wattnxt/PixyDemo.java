package org.firstinspires.ftc.teamcode.wattnxt;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="PixyDemo", group="Pixy")
public class PixyDemo extends OpMode {
    private static final int PORT = 0;
    private static final int ADDRESS = 0x54;
    private static final int DELAY = 20;
    private static final String DIM_NAME = "Pixy dim";

    private PixyCam pixyCam;

    @Override
    public void init() {
        DeviceInterfaceModule dim = hardwareMap.deviceInterfaceModule.get(DIM_NAME);
        pixyCam = new PixyCam(PORT, ADDRESS, DELAY, dim);

        while (!pixyCam.isPortReady()) {
            telemetry.addData("PixyCam", "Waiting for the port to be ready...");
            telemetry.update();
        }
        pixyCam.start();
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

        telemetry.addData("Pixy Cam size", pixyCam.newObjectCount());
        telemetry.addData("New PixyObjects", pixyObjectsString);
        telemetry.update();
    }
}
