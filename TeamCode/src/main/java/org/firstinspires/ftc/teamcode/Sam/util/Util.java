package org.firstinspires.ftc.teamcode.Sam.util;


public final class Util {
    private Util() {
    }

    /*
    * Method blocks until time is elapsed.
    * */
    public static void waitUntil(int timeInMs) {
        try {
            Thread.sleep(timeInMs);
        } catch (InterruptedException ignore) {
            Thread.currentThread().interrupt();
        }
    }


}
