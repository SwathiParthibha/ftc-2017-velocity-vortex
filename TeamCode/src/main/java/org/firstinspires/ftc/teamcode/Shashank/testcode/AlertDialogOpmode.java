package org.firstinspires.ftc.teamcode.Shashank.testcode;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.os.Looper;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Shashank.statemachine.BeaconColor;

/**
 * Created by spmeg on 1/27/2017.
 */
@Autonomous(name = "AlertDialogOpmode", group = "test Opmodes")
public class AlertDialogOpmode extends OpMode {
    private BeaconColor beaconColor = null;

    @Override
    public void init() {
        Activity activity = (Activity) hardwareMap.appContext;
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                AlertDialog.Builder builder = new AlertDialog.Builder(hardwareMap.appContext);
                builder.setTitle(R.string.pickColor)
                        .setItems(R.array.allianceColor, new DialogInterface.OnClickListener() {
                            public void onClick(DialogInterface dialog, int itemPos) {
                                // The 'which' argument contains the index position
                                // of the selected item
                                if(itemPos == 0){
                                    beaconColor = BeaconColor.BLUE;
                                } else {
                                    beaconColor = BeaconColor.RED;
                                }
                            }
                        });
                AlertDialog alertDialog = builder.create();
                telemetry.log().add("alert dialog created");
                telemetry.update();
                alertDialog.show();
            }
        });
        while (beaconColor == null){
            try {
                Thread.sleep(150);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        telemetry.log().add("beacon color"+ beaconColor);
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Beacon color", beaconColor);
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}
