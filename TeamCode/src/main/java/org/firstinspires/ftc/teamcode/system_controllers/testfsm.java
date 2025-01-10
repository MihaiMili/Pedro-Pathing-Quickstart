package org.firstinspires.ftc.teamcode.system_controllers;//package org.firstinspires.ftc.teamcode.system_controllers;
//
//
//
//import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.INITIALIZE;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Globals.robotMap;
//
//@Config
//public class testfsm {
//
//    public enum testStatus
//    {
//        INITIALIZE,
//      STAREA_1,
//        STAREA_2,
//        STAREA_3,
//        STAREA_4,
//        SCORE_DONE,
//        COLLECT_DONE,
//        TRANSFER_DONE,
//    }
//
//    public testfsm()
//    {
//        CS = testStatus.INITIALIZE;
//            PS = testStatus.INITIALIZE;
//    }
//
//    public static testStatus CS = testStatus.INITIALIZE, PS = testStatus.INITIALIZE;
//    ElapsedTime timer = new ElapsedTime();
//    public static double limit = 0.1;
//    public static boolean is_intransfer = false;
//
//    public void update(fourbarController fourbar, clawAngleController clawAngle)
//    {
//        if(CS != PS || CS == testStatus.INITIALIZE || CS == testStatus.STAREA_1 || CS == testStatus.STAREA_2 || CS == testStatus.STAREA_3 || CS == testStatus.COLLECT_DONE || CS == testStatus.SCORE_DONE || CS == testStatus.STAREA_4)
//        {
//            switch (CS)
//            {
//                case STAREA_1:
//                {
//                    clawAngle.CS = clawAngleController.clawAngleStatus.COLLECT_SPECIMEN;
//                    fourbar.CS = fourbarController.fourbarStatus.COLLECT_SPECIMEN;
//                    CS = testStatus.COLLECT_DONE;
//                    break;
//                }
//
//                case STAREA_2:
//                {
//                    is_intransfer = true;
//                    fourbar.CS = fourbarController.fourbarStatus.SPECIMEN_LOW;
//                    timer.reset();
//                    CS = testStatus.STAREA_3;
//                    break;
//                }
//
//                case STAREA_3:
//                {
//                    if(timer.seconds() > limit)
//                    { clawAngle.CS = clawAngleController.clawAngleStatus.SPECIMEN;
//                        is_intransfer = false;
//                    CS = testStatus.TRANSFER_DONE;}
//                            break;
//                }
//
//                case TRANSFER_DONE:
//                {
//                    fourbar.CS = fourbarController.fourbarStatus.SPECIMEN_LOW;
//                    clawAngle.CS = clawAngleController.clawAngleStatus.SPECIMEN;
//                    break;
//
//                }
//
//                case STAREA_4:
//                {
//                    clawAngle.CS = clawAngleController.clawAngleStatus.SPECIMEN_SCORE;
//                    fourbar.CS = fourbarController.fourbarStatus.SCORE_SPECIMEN_LOW;
//                    CS = testStatus.SCORE_DONE;
//                            break;
//                }
//            }
//        }
//
//        PS = CS;
//    }
//
//}
