package org.firstinspires.ftc.teamcode.Auto;


import static org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.DRIVE;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_CLAW;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_COLLECT_ANGLE_BACK;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_DONE;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_EXTENDO;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_OUTTAKE;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_SPIT;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotserver.internal.webserver.controlhubupdater.ChUpdaterCommManager;
import org.firstinspires.ftc.teamcode.Globals.globals;
import org.firstinspires.ftc.teamcode.Globals.robotMap;
import org.firstinspires.ftc.teamcode.system_controllers.clawAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.clawController;
import org.firstinspires.ftc.teamcode.system_controllers.collectAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.extendoController;
import org.firstinspires.ftc.teamcode.system_controllers.fourbarController;
import org.firstinspires.ftc.teamcode.system_controllers.liftController;
import org.firstinspires.ftc.teamcode.system_controllers.outtakeController;

import java.io.BufferedReader;


public class RightAutoController {
    public enum autoControllerStatus
    {
        NOTHING,
        SPECIMEN_START,
        SPECIMEN_ANGLE,
        SPECIMEN_FOURBAR,
        SPECIMEN_DONE,
        SPECIMEN_GRAB,
        SPECIMEN_SCORE,
        SPECIMEN_SCORE_CLAW,
        SPECIMEN_SCORE_DONE,
        SHORT_EXTEND_DONE,
        FULL_EXTEND,
        SHORT_EXTEND,

        EXTEND_DONE,
        RETRACT,
        COLOR_COLLECT,
        COLOR_SPIT,
        COLOR_SPIT_DONE,

        SPECIMEN_SCORE_CYCLE,

        SPECIMEN_SCORE_CYCLE_CLAW,
        COLLECT_SAMPLE_SHORT,
        COLLECT_SAMPLE_LONG,
        COLLECT_SAMPLE_DONE,

        COLLECT_SAMPLE_EXTEND_FULL,


        OUTTAKE_DOWN,
        OPEN_CLAW,
        OPEN_CLAW_DONE,
        OUTTAKE_DOWN_SYSTEMS,
        OUTTAKE_DOWN_FOURBAR,
        OUTTAKE_DOWN_DONE,

        PARK,
        TRANSFER_BEGIN,
        TRANSFER_EXTENDO,
        TRANSFER_SPIT,
        TRANSFER_CLAW,
        TRANSFER_CLAW_CLOSE,
        TRANSFER_OUTTAKE,
        TRANSFER_COLLECT_ANGLE_BACK,
        SAMLPE_START,
        SAMPLE_FOURBAR,
        SAMPLE_SCORE,
        SAMPLE_DONE,
        TRANSFER_DONE,

        FOURBAR_SPECIMEN_UP,
        FOUTBAR_SPECIMEN_UP_DONE,
        FOURBAR_SPECIMEN_DOWN,
        FOURBAR_SPECIMEN_CLOSE_CLAW,
        FOURBAR_SPCIMEN_DONW_DONE,

        OPEN_CLAW_STATUS,
        CLOSE_CLAW_STATUS,

        DONE_CLAW_STATUS,

    }
    public static autoControllerStatus CurrentStatus = autoControllerStatus.NOTHING, PreviousStatus = autoControllerStatus.NOTHING;

    public static autoControllerStatus CurrentStatus2 = autoControllerStatus.NOTHING, PreviousStatus2 = autoControllerStatus.NOTHING;

    public static autoControllerStatus CurrentStatusClaw = autoControllerStatus.NOTHING, PreviousStatusClaw = autoControllerStatus.NOTHING;


    ElapsedTime preloadTimer = new ElapsedTime();
    ElapsedTime extendoTimer = new ElapsedTime();
    ElapsedTime fourbarTimer = new ElapsedTime();
    ElapsedTime clawTimer = new ElapsedTime();
    ElapsedTime cycle_clawTimer = new ElapsedTime();
    ElapsedTime collectAngle_timer = new ElapsedTime();
    ElapsedTime claw_timer = new ElapsedTime();
    ElapsedTime outtake_timer = new ElapsedTime();
    ElapsedTime collect_angle_back = new ElapsedTime();

    ElapsedTime timer = new ElapsedTime();
    public void update(robotMap r, clawAngleController clawAngleController, clawController clawController, collectAngleController collectAngleController, extendoController extendoController, fourbarController fourbarController, liftController liftController )
    {


        switch (CurrentStatus)
        {
            case SPECIMEN_GRAB:
            {
                clawController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawController.clawStatus.CLOSED;
                CurrentStatus = autoControllerStatus.SPECIMEN_START;
                break;
            }
            case SPECIMEN_START:
            {
                liftController.CS = org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.SPECIMEN_HIGH;
                CurrentStatus = autoControllerStatus.SPECIMEN_FOURBAR;
                break;
            }
            case SPECIMEN_FOURBAR:
            {
                fourbarController.CS = org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.SPECIMEN;
                CurrentStatus = autoControllerStatus.SPECIMEN_ANGLE;
                break;
            }
            case SPECIMEN_ANGLE:
            {
                clawAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.SPECIMEN;
                CurrentStatus = autoControllerStatus.SPECIMEN_DONE;
                break;
            }
            case SPECIMEN_SCORE:
            {
                org.firstinspires.ftc.teamcode.system_controllers.liftController.CS = org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.SPECIMEN_SCORE;
                fourbarController.CS = org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.SCORE_SPECIMEN;
                clawAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.SPECIMEN_SCORE;
                clawTimer.reset();
                CurrentStatus = autoControllerStatus.SPECIMEN_SCORE_CLAW;
                break;
            }
            case SPECIMEN_SCORE_CLAW:
            {
                if (clawTimer.seconds() > 0.5)
                {
                    clawController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawController.clawStatus.OPENED;
                    CurrentStatus = autoControllerStatus.SPECIMEN_SCORE_DONE;
                }
                break;
            }

            case OUTTAKE_DOWN:
            {
                org.firstinspires.ftc.teamcode.system_controllers.liftController.CS = org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.DOWN;
                CurrentStatus = autoControllerStatus.OUTTAKE_DOWN_SYSTEMS;
                //outtakeTimer.reset();
                break;
            }
            case OUTTAKE_DOWN_SYSTEMS:
            {
                //if(r.collect.getCurrentPosition() < 500)
                {
                    clawController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawController.clawStatus.COLLECT_SPECIMEN;
                    clawAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.COLLECT_SPECIMEN;
                    fourbarController.CS = org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.INTER;
                    fourbarTimer.reset();
                    CurrentStatus = autoControllerStatus.OUTTAKE_DOWN_FOURBAR;
                }
                break;
            }
            case OUTTAKE_DOWN_FOURBAR:
            {
                if(fourbarTimer.seconds() > 0.3)
                {
                    fourbarController.CS = org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.COLLECT_SPECIMEN;
                    CurrentStatus = autoControllerStatus.OUTTAKE_DOWN_DONE;
                }
                break;
            }

            case COLOR_SPIT:
            {
                r.collect.setPower(0.65);
                CurrentStatus = autoControllerStatus.COLOR_SPIT_DONE;
                break;
            }

            //TODO nush dc nu merge ca e ca la inceput
            case SPECIMEN_SCORE_CYCLE:
            {
                org.firstinspires.ftc.teamcode.system_controllers.liftController.CS = org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.SPECIMEN_SCORE;
                fourbarController.CS = org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.SCORE_SPECIMEN;
                clawAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.SPECIMEN_SCORE;
                //clawController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawController.clawStatus.OPENED;
                cycle_clawTimer.reset();
                CurrentStatus = autoControllerStatus.SPECIMEN_SCORE_CYCLE_CLAW;
                break;
            }

            case SPECIMEN_SCORE_CYCLE_CLAW:
            {
                if (cycle_clawTimer.seconds() > 0.2)
                {
                    clawController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawController.clawStatus.OPENED;
                    CurrentStatus = autoControllerStatus.SPECIMEN_SCORE_DONE;
                }
                break;
            }
            case TRANSFER_BEGIN:
            {
                //globals.is_intransfer = true;
                collectAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.DRIVE;
                //outtakeController.CS = outtakeController.outtakeStatus.TRANSFER_LIFT;
                fourbarController.CS = org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.DRIVE;
                clawAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.DRIVE;
                r.collect.setPower(-1);
                collectAngle_timer.reset();
                CurrentStatus = autoControllerStatus.TRANSFER_EXTENDO;
                break;
            }

            case TRANSFER_EXTENDO:
            {
                if(collectAngle_timer.seconds()>0.3)
                {
                    extendoController.CS = org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.TRANSFER;
                    CurrentStatus = autoControllerStatus.TRANSFER_SPIT;
                }
                break;
            }

            case TRANSFER_SPIT:
            {
                if( r.extendo.getCurrentPosition()*(-1) < 5000)
                {
                    r.collect.setPower(0);
                    claw_timer.reset();
                    CurrentStatus = autoControllerStatus.TRANSFER_CLAW;
                }
                break;
            }

            case TRANSFER_CLAW:
            {
                if(r.extendo.getCurrentPosition()*(-1) < 30 || claw_timer.seconds() > 1)
                {
                    claw_timer.reset();
                    r.collect.setPower(0);
                    CurrentStatus = autoControllerStatus.TRANSFER_CLAW_CLOSE;
                }
                break;
            }

            case TRANSFER_CLAW_CLOSE:
            {
                if (claw_timer.seconds() > 0.2)
                {
                    clawController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawController.clawStatus.CLOSED;
                    outtake_timer.reset();
                    CurrentStatus = autoControllerStatus.TRANSFER_OUTTAKE;
                }
                break;
            }

            case TRANSFER_OUTTAKE:
            {
                if(outtake_timer.seconds() > 0.2)
                {
                    fourbarController.CS = org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.TRANSFER;
                    clawAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.TRANSFER;
                    collectAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.TRANSFER;
                    collect_angle_back.reset();
                    CurrentStatus = autoControllerStatus.TRANSFER_COLLECT_ANGLE_BACK;
                }
                break;
            }


            case TRANSFER_COLLECT_ANGLE_BACK:
            {
                if(collect_angle_back.seconds() > 0.2)
                {
                    collectAngleController.CS= org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.DRIVE;
                    //globals.is_intransfer = false;
                     CurrentStatus = autoControllerStatus.SAMLPE_START;
                }
                break;
            }
            case SAMLPE_START:
            {
                liftController.CS = org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.SAMPLE_HIGH;
                fourbarTimer.reset();
                CurrentStatus = autoControllerStatus.SAMPLE_FOURBAR;
                break;
            }
            case SAMPLE_FOURBAR:
            {
                if (fourbarTimer.seconds() > 0.3)
                {
                   fourbarController.CS = org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.SAMPLE;
                   clawAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.SAMPlE_SCORE;
                   clawTimer.reset();
                   CurrentStatus = autoControllerStatus.SAMPLE_DONE;
                }
                break;
            }
            case SAMPLE_SCORE:
            {
                clawController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawController.clawStatus.OPENED;
                CurrentStatus = autoControllerStatus.SAMPLE_DONE;
                break;
            }

            case FOURBAR_SPECIMEN_UP:
            {
                fourbarController.CS = org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.UP_SPECIMEN;
                CurrentStatus = autoControllerStatus.FOUTBAR_SPECIMEN_UP_DONE;
                break;
            }

            case FOURBAR_SPECIMEN_DOWN:
            {
                fourbarController.CS = org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.COLLECT_SPECIMEN;
                claw_timer.reset();
                CurrentStatus = autoControllerStatus.FOURBAR_SPECIMEN_CLOSE_CLAW;
                break;
            }

            case FOURBAR_SPECIMEN_CLOSE_CLAW:
            {
                if(claw_timer.seconds() > 0.15)
                {
                    clawController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawController.clawStatus.CLOSED;
                    CurrentStatus = autoControllerStatus.FOURBAR_SPCIMEN_DONW_DONE;
                }
                break;
            }
        }

        //todo switch between cs2 and cs1

        switch (CurrentStatus2)
        {
            //nush
            case SHORT_EXTEND:
            {
                extendoController.CS = org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.SHORT_EXTEND_SPECIMEN;
                collectAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.COLLECT;
                //r.collect.setPower(-1);
                CurrentStatus2 = autoControllerStatus.SHORT_EXTEND_DONE;
                break;
            }

            case FULL_EXTEND:
            {
                extendoController.CS = org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.EXTENDED_SPECIMEN;
                //collectAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.COLLECT;
                //r.collect.setPower(-1);
                CurrentStatus2 = autoControllerStatus.EXTEND_DONE;
                break;
            }

            case COLLECT_SAMPLE_SHORT:
            {
                extendoController.CS = org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.SHORT;
                collectAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.COLLECT;
                r.collect.setPower(-1);
                CurrentStatus2 = autoControllerStatus.COLLECT_SAMPLE_DONE;
                break;
            }
            case COLLECT_SAMPLE_LONG:
            {
                    extendoController.CS = org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.EXTENDED_SPECIMEN;
                    CurrentStatus2 = autoControllerStatus.COLLECT_SAMPLE_DONE;
                    break;
            }
            case PARK:
            {
                extendoController.CS = org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.EXTENDED;
                liftController.CS = org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.DOWN;
                fourbarController.CS = org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.INTER;
                CurrentStatus2 = autoControllerStatus.EXTEND_DONE;
                break;
            }
            case RETRACT:
            {
                r.collect.setPower(0);
                extendoController.CS = org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.RETRACTED;
                collectAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.DRIVE;
                CurrentStatus2 = autoControllerStatus.NOTHING;
                break;

            }

        }

        switch (CurrentStatusClaw)
        {
            case OPEN_CLAW:
            {
                clawController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawController.clawStatus.OPENED;
                CurrentStatusClaw = autoControllerStatus.DONE_CLAW_STATUS;
                break;
            }

            case CLOSE_CLAW_STATUS:
            {
                clawController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawController.clawStatus.CLOSED;
                CurrentStatusClaw = autoControllerStatus.DONE_CLAW_STATUS;
                break;
            }
        }

        PreviousStatusClaw = CurrentStatusClaw;
        PreviousStatus = CurrentStatus;
        PreviousStatus2 = CurrentStatus2;
    }
}