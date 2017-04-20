package org.firstinspires.ftc.teamcode.wattnxt.refactored;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class PixyObject {
    public PixyObject(short checksum, short signature, short centerX, short centerY, short width, short height) {
        this.checksum = checksum;
        this.signature = signature;
        this.centerX = centerX;
        this.centerY = centerY;
        this.width = width;
        this.height = height;
    }

    @Override
    public String toString() {
        return "Sig: " + signature +
                ", Center: (" + centerX + ", " + centerY +
                "), Width: " + width +
                ", Height: " + height;
    }

    private static final short SYNC_WORD = (short) 0xaa55;

    public short checksum;
    public short signature;
    public short centerX;
    public short centerY;
    public short width;
    public short height;

    public static PixyObject fromBytes(byte[] data) {
        /*
        Bytes    16-bit words   Description
        -------------------------------------------------------------
        0, 1     0              sync (0xaa55)
        2, 3     1              checksum (sum of all 16-bit words 2-6)
        4, 5     2              signature number
        6, 7     3              x center of object
        8, 9     4              y center of object
        10, 11   5              width of object
        12, 13   6              height of object
        */
        if (data.length != 14) {
            return null;
        }

        ByteBuffer byteBuffer = ByteBuffer.wrap(data);
        byteBuffer.order(ByteOrder.LITTLE_ENDIAN);

        short syncWord = byteBuffer.getShort();
        if (syncWord != SYNC_WORD) {
            return null;
        }

        short checksum = byteBuffer.getShort();

        short signature = byteBuffer.getShort();
        short centerX = byteBuffer.getShort();
        short centerY = byteBuffer.getShort();
        short width = byteBuffer.getShort();
        short height = byteBuffer.getShort();

        if (checksum != signature + centerX + centerY + width + height) {
            return null;
        }

        return new PixyObject(checksum, signature, centerX, centerY, width, height);
    }
}
