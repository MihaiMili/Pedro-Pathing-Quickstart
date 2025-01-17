package org.firstinspires.ftc.teamcode.system_controllers;



import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.COLLECT_SPECIMEN;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.COLLECT_SPECIMEN_2;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.COLLECT_SPECIMEN_3;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.COLLECT_SPECIMEN_DONE;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.INTER;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.INTER_SPECIMEN_GEN;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.SAMPLE_HIGH;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.SAMPLE_HIGH_FOURBAR;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.SAMPLE_LOW;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.SCORE_SPECIMEN_CLAW;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.SPECIMEN_HIGH;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.SPECIMEN_LOW;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.TIMER_VERIF;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.TRANSFER_CLAW_ANGLE;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.TRANSFER_DONE;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.TRANSFER_LIFT;
import static org.firstinspires.ftc.teamcode.system_controllers.outtakeController.outtakeStatus.TRANSFER_SYSTEMS;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Globals.globals;
import org.firstinspires.ftc.teamcode.Globals.robotMap;

@Config
public class outtakeController {

    public enum outtakeStatus
    {
        INITIALIZE,
        TRANSFER_LIFT,
        TRANSFER_SYSTEMS,
        SPECIMEN_HIGH,
        SPECIMEN_LOW,
        SAMPLE_HIGH,
        SAMPLE_HIGH_FOURBAR,
        SAMPLE_LOW,

        COLLECT_SPECIMEN,
        COLLECT_SPECIMEN_2,
        COLLECT_SPECIMEN_3,
        COLLECT_SPECIMEN_DONE,
        TRANSFER_DONE,
        SCORE_SPECIMEN,
        SCORE_SPECIMEN_CLAW,
        INTER,
        TIMER_VERIF,
        TRANSFER_CLAW_ANGLE,
        HANG_LV1,
        HANG_LV2,
        HANG_LIL_DOWN,
        HANG_DOWN,
        INTER_SPECIMEN_GEN,
    }

    public outtakeController()
    {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static outtakeStatus CS = INITIALIZE, PS = INITIALIZE;
    ElapsedTime timer_drive = new ElapsedTime();
    public static double timer_limit =0.3;
    public static double claw_limit =0.3;
    public static double colect_specimen_limit = 0.3;
    public static double colect_specimen_inter_limit = 0.2;
    public static double inter_limit = 0.15;
    public static double claw_anlgle_limit = 0.1;
    ElapsedTime collect_timer = new ElapsedTime();
    ElapsedTime timer_verif = new ElapsedTime();
    ElapsedTime collect_inter_timer = new ElapsedTime();
    ElapsedTime claw_angle_timer = new ElapsedTime();
    ElapsedTime fourbar_timer = new ElapsedTime();

    public void update(robotMap r, clawController claw, liftController lift, fourbarController fourbar, clawAngleController clawAngle, transferController transfer)
    {
        if(CS != PS || CS == INITIALIZE || CS == outtakeStatus.SCORE_SPECIMEN || CS == TRANSFER_CLAW_ANGLE || CS == COLLECT_SPECIMEN_2 || CS == COLLECT_SPECIMEN_DONE|| CS == COLLECT_SPECIMEN_2 || CS == SCORE_SPECIMEN_CLAW || CS == outtakeStatus.TRANSFER_LIFT || CS == TRANSFER_SYSTEMS || CS == TRANSFER_SYSTEMS || CS == SPECIMEN_HIGH || CS == SPECIMEN_LOW || CS == SAMPLE_HIGH || CS == SAMPLE_LOW || CS == COLLECT_SPECIMEN  || CS == TRANSFER_DONE || CS == INTER || CS == TIMER_VERIF || CS == COLLECT_SPECIMEN_3 || CS == INTER_SPECIMEN_GEN || CS == SAMPLE_HIGH_FOURBAR)
        {
            switch (CS)
            {

                case INTER_SPECIMEN_GEN:
                {
                    fourbar.CS = fourbarController.fourbarStatus.INTER;
                    lift.CS = liftController.liftStatus.DOWN;
                    timer_verif.reset();
                    CS = TIMER_VERIF;
                    break;

                }

                case INTER:
                {
                   if(timer_verif.seconds() > 0.1)
                   {
                       CS = TRANSFER_LIFT;
                   }
                    break;
                }

                case TIMER_VERIF:
                {
                    if(timer_verif.seconds()>inter_limit)
                    {
                        globals.is_scoring_specimen = false;
                        CS = TRANSFER_LIFT;
                    }
                    break;
                }

                case TRANSFER_LIFT:
                {
                    fourbar.CS = fourbarController.fourbarStatus.DRIVE;
                    timer_drive.reset();
                    CS = TRANSFER_SYSTEMS;
                    break;
                }

                case TRANSFER_SYSTEMS:
                {
                    if(r.collect.getCurrentPosition() < 350 || timer_drive.seconds() > 0.1)
                    {
                        claw.CS = clawController.clawStatus.OPENED;
                        clawAngle.CS = clawAngleController.clawAngleStatus.DRIVE;
                        claw_angle_timer.reset();
                        CS = TRANSFER_CLAW_ANGLE;
                    }
                    break;
                }

                case TRANSFER_CLAW_ANGLE:
                {
                    if(claw_angle_timer.seconds() > claw_anlgle_limit)
                    {
                        if (globals.is_intransfer == false) lift.CS = liftController.liftStatus.DOWN;
                            else lift.CS = liftController.liftStatus.TRANSFER;
                        CS = TRANSFER_DONE;
                    }
                    break;
                }

                case SPECIMEN_HIGH:
                {
                    lift.CS = liftController.liftStatus.SPECIMEN_HIGH;
                    fourbar.CS = fourbarController.fourbarStatus.SPECIMEN;
                    //clawAngle.CS = clawAngleController.clawAngleStatus.SPECIMEN;
                    break;
                }

                case SPECIMEN_LOW:
                {
                    lift.CS = liftController.liftStatus.SPECIMEN_LOW;
                    fourbar.CS = fourbarController.fourbarStatus.SPECIMEN;
                    clawAngle.CS = clawAngleController.clawAngleStatus.SPECIMEN;
                    break;
                }

                case SAMPLE_LOW:
                {
                    lift.CS = liftController.liftStatus.SAMPLE_LOW;
                    fourbar.CS = fourbarController.fourbarStatus.SAMPLE;
                    clawAngle.CS = clawAngleController.clawAngleStatus.SAMPlE_SCORE;
                    break;
                }

                case SAMPLE_HIGH:
                {
                    lift.CS = liftController.liftStatus.SAMPLE_HIGH;
                    //fourbar.CS = fourbarController.fourbarStatus.SAMPLE;
                    fourbar_timer.reset();
                    CS = SAMPLE_HIGH_FOURBAR;
                    break;
                }

                case SAMPLE_HIGH_FOURBAR:
                {
                    //if (r.lift.getCurrentPosition() > 43000)
                    if (fourbar_timer.seconds() > 0.6)
                    {
                        fourbar.CS = fourbarController.fourbarStatus.SAMPLE;
                        clawAngle.CS = clawAngleController.clawAngleStatus.SAMPlE_SCORE;
                    }
                    break;
                }
                case COLLECT_SPECIMEN:
                {
                    //if(timer_drive.seconds() > claw_limit) {
                        lift.CS = liftController.liftStatus.DOWN;
                        // clawController.CS = clawController.clawStatus.OPENED;
                        // fourbar.CS = fourbarController.fourbarStatus.COLLECT_SPECIMEN_INTER;
                        collect_timer.reset();
                        CS = COLLECT_SPECIMEN_2;
                    //}
                    break;
                }

                case COLLECT_SPECIMEN_2:
                {
                        clawAngle.CS = clawAngleController.clawAngleStatus.COLLECT_SPECIMEN;
                        collect_inter_timer.reset();
                        CS = COLLECT_SPECIMEN_3;
                    break;
                }

                case COLLECT_SPECIMEN_3:
                {
                        fourbar.CS = fourbarController.fourbarStatus.COLLECT_SPECIMEN;
                        CS = COLLECT_SPECIMEN_DONE;
                        break;
                }

                case SCORE_SPECIMEN:
                {
                    fourbar.CS = fourbarController.fourbarStatus.SCORE_SPECIMEN;
                    clawAngle.CS = clawAngleController.clawAngleStatus.SPECIMEN_SCORE;
                    lift.CS = liftController.liftStatus.SPECIMEN_SCORE;
                    timer_drive.reset();
                    globals.is_scoring_specimen = true;
                    CS = SCORE_SPECIMEN_CLAW;
                    break;
                }

                case SCORE_SPECIMEN_CLAW:
                {
                    if(timer_drive.seconds()> claw_limit)
                    {
                        claw.CS = clawController.clawStatus.OPENED;
                    }
                    if(timer_drive.seconds() > 0.6)
                    {
                        CS = INTER_SPECIMEN_GEN;
                    }
                    break;
                }

                case HANG_LV1:
                {
                    lift.CS = liftController.liftStatus.HANG_LV1;
                    clawAngle.CS = clawAngleController.clawAngleStatus.HANG_LV1;
                    fourbar.CS = fourbarController.fourbarStatus.HANG_LV1;
                    break;
                }

                case HANG_LV2:
                {
                    lift.CS = liftController.liftStatus.HANG_LV2;
                    clawAngle.CS = clawAngleController.clawAngleStatus.HANG_LV2;
                    fourbar.CS = fourbarController.fourbarStatus.HANG_LV2;
                    break;
                }

                case HANG_LIL_DOWN:
                {
                    lift.CS = liftController.liftStatus.HANG_LIL_DOWN;
                    clawAngle.CS = clawAngleController.clawAngleStatus.HANG_LV2;
                    fourbar.CS = fourbarController.fourbarStatus.HANG_LV2;
                    break;
                }

                case HANG_DOWN:
                {
                    lift.CS = liftController.liftStatus.TRANSFER;
                    clawAngle.CS = clawAngleController.clawAngleStatus.HANG_LV2;
                    fourbar.CS = fourbarController.fourbarStatus.HANG_LV2;
                    break;
                }
            }
        }

        PS = CS;
    }

}
