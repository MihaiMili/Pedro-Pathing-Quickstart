package org.firstinspires.ftc.teamcode.system_controllers;//package org.firstinspires.ftc.teamcode.system_controllers;
//
//
//import com.acmerobotics.dashboard.config.Config;
//
//import org.firstinspires.ftc.teamcode.Globals.robotMap;
//
//@Config
//public class ruletaController {
//
//    public enum ruletaStatus
//    {
//        INITIALIZE,
//        ON,
//        OFF,
//    }
//
//    public ruletaController()
//    {
//        CS = ruletaStatus.INITIALIZE;
//        PS = ruletaStatus.INITIALIZE;
//    }
//
//    public static ruletaStatus CS = ruletaStatus.INITIALIZE, PS = ruletaStatus.INITIALIZE;
//
//    public static double on = 0;
//    public static double off = 1;
//
//    public void update(robotMap r)
//    {
//        if(CS != PS || CS == ruletaStatus.INITIALIZE || CS == ruletaStatus.ON || CS == ruletaStatus.OFF)
//        {
//            switch (CS)
//            {
//                case INITIALIZE:
//                {
//                    r.ruleta.setPosition(off);
//                    break;
//                }
//
//                case ON:
//                {
//                    r.ruleta.setPosition(on);
//                    break;
//                }
//
//                case OFF:
//                {
//                    r.ruleta.setPosition(off);
//                    break;
//                }
//
//            }
//        }
//
//        PS = CS;
//    }
//
//}
