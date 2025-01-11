package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.COLLECT;
import static org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.DONE;
import static org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.DRIVE;
import static org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.SWEEP;
import static org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.TRANSFER;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Globals.robotMap;

@Config
public class collectAngleController {

    public enum collectAngleStatus
    {
        INITIALIZE,
        COLLECT,
        TRANSFER,
        SWEEP,
        DONE,
        DRIVE,
    }

    public collectAngleController()
    {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static collectAngleStatus CS = INITIALIZE, PS = INITIALIZE;

    public static double collect = 0.45;
    public static double drive = 0.95;
    public static double transfer = 0.875;
    public static double maturare = 0.63;
    public void update(robotMap r)
    {
         if(CS != PS || CS == INITIALIZE || CS == COLLECT || CS == DRIVE || CS == TRANSFER || CS == DONE || CS == SWEEP)
         {
             switch (CS)
             {
                 case INITIALIZE:
                 {
                     r.collectAngle.setPosition(drive);
                     break;
                 }

                 case COLLECT:
                 {
                     r.collectAngle.setPosition(collect);
                     break;
                 }

                 case DRIVE:
                 {
                     r.collectAngle.setPosition(drive);
                     CS = DONE;
                     break;
                 }
                 case TRANSFER:
                 {
                     r.collectAngle.setPosition(transfer);
                     break;
                 }
                 case SWEEP:
                 {
                     r.collectAngle.setPosition(maturare);
                     break;
                 }
             }

         }

         PS = CS;
    }

}
