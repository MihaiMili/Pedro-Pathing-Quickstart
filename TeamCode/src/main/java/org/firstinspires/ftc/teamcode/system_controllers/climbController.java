package org.firstinspires.ftc.teamcode.system_controllers;//package org.firstinspires.ftc.teamcode.system_controllers;
//
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Globals.robotMap;
//
//public class climbController {
//
//    public enum climbStatus{
//        INITIALIZE,
//        HANG_UP,
//        HANG_BEGIN,
//        HANG_DOWN,
//        DONE,
//    }
//
//    public climbController()
//    {
//        CS = climbStatus.INITIALIZE;
//        PS = climbStatus.INITIALIZE;
//    }
//
//    public static climbStatus CS = climbStatus.INITIALIZE, PS = climbStatus.INITIALIZE;
//    ElapsedTime hang_timer = new ElapsedTime();
//
//    public void update(robotMap r, outtakeController outtake, ptoController pto, ruletaController ruleta)
//    {
//        if(CS != PS || CS == climbStatus.INITIALIZE || CS == climbStatus.DONE)
//        {
//            switch (CS)
//            {
//                case HANG_UP:
//                {
//                    outtake.CS = outtakeController.outtakeStatus.HANG_LV2;
//                    pto.CS = ptoController.ptoStatus.ON;
//                    ruleta.CS = ruletaController.ruletaStatus.ON;
//                    break;
//                }
//
//                case HANG_BEGIN:
//                {
//                    outtake.CS = outtakeController.outtakeStatus.HANG_LIL_DOWN;
//                    ruleta.CS = ruletaController.ruletaStatus.OFF;
//                    hang_timer.reset();
//                    CS = climbStatus.HANG_DOWN;
//                    break;
//                }
//
//                case HANG_DOWN:
//                {
//                    if(hang_timer.seconds() > 3)
//                    {
//                        outtake.CS = outtakeController.outtakeStatus.HANG_DOWN;
//                        CS = climbStatus.DONE;
//                    }
//                }
//            }
//        }
//
//        PS = CS;
//    }
//}
