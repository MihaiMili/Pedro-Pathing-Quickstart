package org.firstinspires.ftc.teamcode.system_controllers;




import static org.firstinspires.ftc.teamcode.system_controllers.intakeController.intakeStatus.EXTENDO_DONE_LONG;
import static org.firstinspires.ftc.teamcode.system_controllers.intakeController.intakeStatus.EXTENDO_DONE_SHORT;
import static org.firstinspires.ftc.teamcode.system_controllers.intakeController.intakeStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.intakeController.intakeStatus.RETRACT_COLLECT;
import static org.firstinspires.ftc.teamcode.system_controllers.intakeController.intakeStatus.RETRACT_DONE;
import static org.firstinspires.ftc.teamcode.system_controllers.intakeController.intakeStatus.RETRACT_EXTENDO;
import static org.firstinspires.ftc.teamcode.system_controllers.intakeController.intakeStatus.SHORT;
import static org.firstinspires.ftc.teamcode.system_controllers.intakeController.intakeStatus.LONG;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Globals.globals;
import org.firstinspires.ftc.teamcode.Globals.robotMap;

@Config
public class intakeController {

    public enum intakeStatus
    {
        INITIALIZE,
        SHORT,
        LONG,
        RETRACT_COLLECT,
        RETRACT_EXTENDO,
        RETRACT_DONE,
        EXTENDO_DONE_SHORT,
        EXTENDO_DONE_LONG,
    }

    public intakeController()
    {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static intakeStatus CS = INITIALIZE, PS = INITIALIZE;
    public static double collect_limit = 0.5;
    ElapsedTime timer_collect = new ElapsedTime();

    public void update(robotMap r, extendoController extendo)
    {
        if(CS != PS || CS == INITIALIZE || CS == RETRACT_EXTENDO || CS == RETRACT_COLLECT || CS == SHORT || CS == LONG || CS == EXTENDO_DONE_LONG|| CS == RETRACT_DONE || CS == EXTENDO_DONE_SHORT)
        {
           switch (CS)
           {
               case INITIALIZE:
               {
                   extendo.CS = extendoController.extendoStatus.RETRACTED;
                   collectAngleController.CS = collectAngleController.collectAngleStatus.DRIVE;
                   break;
               }
               case SHORT:
               {
                   extendo.CS = extendoController.extendoStatus.SHORT;
                   CS = intakeStatus.EXTENDO_DONE_SHORT;
                   break;
               }
               case LONG:
               {
                   extendo.CS = extendoController.extendoStatus.EXTENDED;
                   CS = EXTENDO_DONE_LONG;
                   break;
               }
               case RETRACT_COLLECT:
               {
                   if(collectAngleController.CS != collectAngleController.collectAngleStatus.DRIVE)
                   { timer_collect.reset();
                       collect_limit = 0.2;
                   collectAngleController.CS = collectAngleController.collectAngleStatus.DRIVE;
                       CS = RETRACT_EXTENDO;}
                 else  {
                     timer_collect.reset();
                     collect_limit =0;
                   CS = RETRACT_EXTENDO;}
                   break;
               }
               case RETRACT_EXTENDO:
               {
                   if (timer_collect.seconds() >= collect_limit)
                   {
                       extendo.CS = extendoController.extendoStatus.RETRACTED;
                       CS = RETRACT_DONE;
                   }
                   break;
               }
           }
        }
        PS = CS;
    }

}
