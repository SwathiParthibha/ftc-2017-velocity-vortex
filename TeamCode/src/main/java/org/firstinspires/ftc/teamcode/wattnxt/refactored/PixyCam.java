package org.firstinspires.ftc.teamcode.wattnxt.refactored;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.locks.Lock;

public class PixyCam implements Runnable {
    private int port;
    private I2cAddr address;
    private DeviceInterfaceModule dim;

    private byte[] readCache;
    private Lock readLock;

    private List<Byte> rawData;
    private List<PixyObject> pixyObjects;

    public PixyCam(int port, int address, DeviceInterfaceModule dim) {
        this.port = port;
        this.address = I2cAddr.create7bit(address);
        this.dim = dim;

        readCache = dim.getI2cReadCache(port);
        readLock = dim.getI2cReadCacheLock(port);

        rawData = new ArrayList<>();
        pixyObjects = Collections.synchronizedList(new ArrayList<PixyObject>());

    }

    public void init(){
        /*this updates the cache given from the controller
        The method dim.writeI2cCacheToController(port) updates the cache, by writing to the dim
         */
        dim.enableI2cReadMode(port, this.address, 0, 16);
        dim.setI2cPortActionFlag(port);
        dim.writeI2cCacheToController(port);
    }

    public boolean isPortReady() {
        return dim.isI2cPortReady(port);
    }

    @Override
    public void run() {
        for (int i = 0; i < 40; i++) {
            init();
            readI2cCache();
            DbgLog.msg("> PIXY: RAWDATA SIZE: " + rawData.size());
            DbgLog.msg("> PIXY: RAWDATA: " + rawData.toString());
            DbgLog.msg("> PIXY: READCACHE: " + Arrays.toString(readCache));
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
                pixyObjects.add(0, pixyObject);
                rawData = rawData.subList(14, rawData.size());
            }
        }
    }

    private void readI2cCache() {
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
