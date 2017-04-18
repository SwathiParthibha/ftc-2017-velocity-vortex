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
    private static Integer COUNTER = 0;

    public PixyCam(int port, int address, DeviceInterfaceModule dim) {
        this.port = port;
        this.address = I2cAddr.create7bit(address);
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
        PixyCam.COUNTER++;
        DbgLog.msg("> PIXY THREAD: THREAD IS ON ITS " + COUNTER + "nth TIME");
        /*this updates the cache given from the controller
        The method dim.writeI2cCacheToController(port) updates the cache, by writing to the dim

        The port action is flag is used to tell the controller to execute the transaction
        in it's buffer. The controller clears the flag when the transaction is complete,
        which lets software know that it can write another transaction and set the
        port action flag again.
         */
        DbgLog.msg("> PIXY THREAD: PORT ACTION FLAG IS " + dim.isI2cPortActionFlagSet(port));
        if(!dim.isI2cPortActionFlagSet(port))
            return;

        dim.setI2cPortActionFlag(port);

        dim.writeI2cCacheToController(port);

        /*by telling to enable read mode, we are
         writing to the controller and telling it
         to read from a specific spot
          */
        dim.enableI2cReadMode(port, this.address, 0, 16);
        //updates the read cache
        dim.readI2cCacheFromController(port);

        try {
            readLock.lock();
            DbgLog.msg("> PIXY THREAD: SIZE OF READ CHACHE IS: " + readCache.length);
            DbgLog.msg("> PIXY THREAD: READCACHE IS: " + Arrays.toString(readCache));
            // Value 0..3 is Mode, Address, Location, Length.
            // The data bytes we want come after.
            //First four bytes are metadata
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
