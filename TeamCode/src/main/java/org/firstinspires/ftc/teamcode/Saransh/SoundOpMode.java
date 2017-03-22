package org.firstinspires.ftc.teamcode.Saransh;

import android.app.Activity;
import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.R;

/**
 * Created by spmeg on 1/27/2017.
 */
@Autonomous(name = "Gotta Catch Em All Test", group = "test op")
@Disabled
public class SoundOpMode extends OpMode {
    private MediaPlayer mediaPlayer = null;
    @Override
    public void init() {
        Activity activity = (Activity) this.hardwareMap.appContext;
        mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.gotta_catch_em_all);
        mediaPlayer.start();
        telemetry.log().add("duration: " + mediaPlayer.getDuration());
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("is Playing", mediaPlayer.isPlaying());
        telemetry.addData("Current position: ",mediaPlayer.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
        mediaPlayer.stop();
        mediaPlayer.release();
        mediaPlayer = null;
    }
}
