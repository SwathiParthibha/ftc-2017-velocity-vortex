package org.firstinspires.ftc.teamcode.wattnxt;

import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.locks.Lock;

public class PixyCam extends Thread {
    private int port;
    private I2cAddr address;
    private int delay;
    private DeviceInterfaceModule dim;

    private byte[] readCache;
    private Lock readLock;

    private List<Byte> rawData;
    private List<PixyObject> pixyObjects;

    public PixyCam(int port, int address, int delay, DeviceInterfaceModule dim) {
        this.port = port;
        this.address = I2cAddr.create7bit(address);
        this.delay = delay;
        this.dim = dim;

        readCache = dim.getI2cReadCache(port);
        readLock = dim.getI2cReadCacheLock(port);

        rawData = new ArrayList<>();
        pixyObjects = Collections.synchronizedList(new ArrayList<PixyObject>());
    }

    public boolean isPortReady() {
        return dim.isI2cPortReady(port);
    }

    @Override
    public void run() {
        while (true) {
            dim.enableI2cReadMode(port, address, 0, 16);
            dim.setI2cPortActionFlag(port);
            dim.writeI2cCacheToController(port);

            dim.readI2cCacheFromController(port);
            try {
                readLock.lock();
                // Value 0..3 is Mode, Address, Location, Length.
                // The data bytes we want come after.
                for (int i = 4; i < 4+16; i++) {
                    rawData.add(readCache[i]);
                }
            } finally {
                readLock.unlock();
            }

            while (rawData.size() >= 14) {
                Byte[] segment = rawData.subList(0, 14).toArray(new Byte[14]);
                byte[] segmentPrimative = new byte[14];
                for (int i = 0; i < 14; i++) {
                    segmentPrimative[i] = segment[i];
                }
                PixyObject pixyObject = PixyObject.fromBytes(segmentPrimative);
                if (pixyObject == null) {
                    rawData.remove(0);
                } else {
                    pixyObjects.add(pixyObject);
                    rawData = rawData.subList(14, rawData.size());
                }
            }
            try {
                Thread.sleep(delay);
            } catch (InterruptedException e) {
                break;
            }
        }
    }

    public int newObjectCount() {
        return pixyObjects.size();
    }

    public boolean hasNewObject() {
        return pixyObjects.size() > 0;
    }

    public PixyObject popObject() {
        if (pixyObjects.size() == 0) {
            return null;
        }
        return pixyObjects.remove(0);
    }
}
