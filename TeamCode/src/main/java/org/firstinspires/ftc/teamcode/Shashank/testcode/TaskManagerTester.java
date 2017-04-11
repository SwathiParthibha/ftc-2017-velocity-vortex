package org.firstinspires.ftc.teamcode.Shashank.testcode;

import com.qualcomm.ftccommon.DbgLog;

import trclib.TrcRobot;
import trclib.TrcTaskMgr;

/**
 * Created by spmeg on 4/1/2017.
 */

public class TaskManagerTester {

    public static void main(String[] args){
        TrcTaskMgr taskMgr = new TrcTaskMgr();
        TrcTaskMgr.Task task = new TrcTaskMgr.Task() {
            @Override
            public void startTask(TrcRobot.RunMode runMode) {
                System.out.println("START TASK");
            }

            @Override
            public void stopTask(TrcRobot.RunMode runMode) {
                System.out.println("STOP TASK");
            }

            @Override
            public void prePeriodicTask(TrcRobot.RunMode runMode) {
                System.out.println("prePeriodicTask TASK");
            }

            @Override
            public void postPeriodicTask(TrcRobot.RunMode runMode) {
                System.out.println("postPeriodicTask TASK");
            }

            @Override
            public void preContinuousTask(TrcRobot.RunMode runMode) {
                System.out.println("preContinuousTask TASK");
            }

            @Override
            public void postContinuousTask(TrcRobot.RunMode runMode) {
                System.out.println("postContinuousTask TASK");
            }
        };


        taskMgr.registerTask("task whatevr1", task, TrcTaskMgr.TaskType.START_TASK);
        taskMgr.registerTask("task whatevr2", task, TrcTaskMgr.TaskType.PREPERIODIC_TASK);
        taskMgr.registerTask("task whatevr3", task, TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
        taskMgr.registerTask("task whatevr4", task, TrcTaskMgr.TaskType.POSTPERIODIC_TASK);
        taskMgr.registerTask("task whatevr5", task, TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        taskMgr.registerTask("task whatevr6", task, TrcTaskMgr.TaskType.STOP_TASK);

        taskMgr.executeTaskType(TrcTaskMgr.TaskType.START_TASK, TrcRobot.RunMode.AUTO_MODE);
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.PREPERIODIC_TASK, TrcRobot.RunMode.AUTO_MODE);
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK, TrcRobot.RunMode.AUTO_MODE);
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.POSTPERIODIC_TASK, TrcRobot.RunMode.AUTO_MODE);
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK, TrcRobot.RunMode.AUTO_MODE);
        taskMgr.executeTaskType(TrcTaskMgr.TaskType.STOP_TASK, TrcRobot.RunMode.AUTO_MODE);
    }

}
