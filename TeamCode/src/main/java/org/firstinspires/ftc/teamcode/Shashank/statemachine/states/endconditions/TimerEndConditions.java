package org.firstinspires.ftc.teamcode.Shashank.statemachine.states.endconditions;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

import ftc.electronvolts.statemachine.EndCondition;
import ftc.electronvolts.statemachine.EndConditions;

/**
 * Created by spmeg on 4/13/2017.
 */

public class TimerEndConditions extends EndConditions {
    public static EndCondition timerMilli(final int time){
        return new EndCondition() {
            private ElapsedTime elapsedTime = new ElapsedTime();

            @Override
            public void init() {
                elapsedTime.reset();
            }

            @Override
            public boolean isDone() {
                return elapsedTime.milliseconds() > time;
            }
        };
    }

    public static EndCondition timerSeconds(final int time){
        return new EndCondition() {
            private ElapsedTime elapsedTime = new ElapsedTime();

            @Override
            public void init() {
                elapsedTime.reset();
            }

            @Override
            public boolean isDone() {
                return elapsedTime.seconds() > time;
            }
        };
    }
}
