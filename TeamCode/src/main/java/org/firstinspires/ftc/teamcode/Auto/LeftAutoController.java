package org.firstinspires.ftc.teamcode.Auto;


import static org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.COLLECT;
import static org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.DRIVE;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_CLAW;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_COLLECT_ANGLE_BACK;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_DONE;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_EXTENDO;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_OUTTAKE;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_SPIT;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Globals.globals;
import org.firstinspires.ftc.teamcode.Globals.robotMap;
import org.firstinspires.ftc.teamcode.system_controllers.clawAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.clawController;
import org.firstinspires.ftc.teamcode.system_controllers.collectAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.extendoController;
import org.firstinspires.ftc.teamcode.system_controllers.fourbarController;
import org.firstinspires.ftc.teamcode.system_controllers.liftController;
import org.firstinspires.ftc.teamcode.system_controllers.outtakeController;


public class LeftAutoController {
    public enum autoControllerStatus
    {
        NOTHING,
        SAMPLE_START,
        SAMPLE_FOURBAR,

        COLLECT_SAMPLE,
        COLLECT_SAMPLE_ANGLE,
        COLLECT_SAMPLE_DONE,
        COLLECT_SAMPLE_EXTEND_FULL,


        OUTTAKE_DOWN,
        OPEN_CLAW,
        OPEN_CLAW_DONE,
        OUTTAKE_DOWN_SYSTEMS,
        OUTTAKE_DOWN_FOURBAR,
        OUTTAKE_DOWN_DONE,

        COLLECT_SUMMERSIBLE_SHORT,
        COLLECT_SUMMERSIBLE_EXTEND_SHORT,

        COLLECT_SUMMERSIBLE_EXTEND_LONG,
        SUMMERSIBLE_RETRACT,
        COLLECT_SUMMERSIBLE_DONE,

        RETRACT_SHORT,

        TRANSFER_BEGIN,
        TRANSFER_EXTENDO,
        TRANSFER_SPIT,
        TRANSFER_CLAW,
        TRANSFER_CLAW_CLOSE,
        TRANSFER_OUTTAKE,
        TRANSFER_COLLECT_ANGLE_BACK,
        TRANSFER_DONE,

        SAMPLE_DONE,
        HANG_LV1

    }
    public static autoControllerStatus CurrentStatus = autoControllerStatus.NOTHING, PreviousStatus = autoControllerStatus.NOTHING;

    public static autoControllerStatus CurrentStatus2 = autoControllerStatus.NOTHING, PreviousStatus2 = autoControllerStatus.NOTHING;


    ElapsedTime preloadTimer = new ElapsedTime();
    ElapsedTime extendoTimer = new ElapsedTime();
    ElapsedTime fourbarTimer = new ElapsedTime();
    ElapsedTime claw_timer = new ElapsedTime();
    ElapsedTime collectAngle_timer = new ElapsedTime();
    ElapsedTime outtake_timer = new ElapsedTime();


    ElapsedTime collect_angle_back = new ElapsedTime();

    public void update(robotMap r, clawAngleController clawAngleController, clawController clawController, collectAngleController collectAngleController, extendoController extendoController, fourbarController fourbarController, liftController liftController )
    {

        switch (CurrentStatus)
        {


            case SAMPLE_START:
            {

                liftController.CS = org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.SAMPLE_HIGH;
                preloadTimer.reset();
                CurrentStatus = autoControllerStatus.SAMPLE_FOURBAR;
                break;
            }

            case SAMPLE_FOURBAR:
            {
                if(preloadTimer.seconds() > 0.5)
                {
                    fourbarController.CS = org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.SAMPLE;
                    clawAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.SAMPlE_SCORE;
                    CurrentStatus = autoControllerStatus.SAMPLE_DONE;
                }
                break;
            }

            case COLLECT_SAMPLE:
            {
                extendoController.CS = org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.SHORT;
                CurrentStatus = autoControllerStatus.COLLECT_SAMPLE_ANGLE;
                break;
            }

            case COLLECT_SAMPLE_ANGLE:
            {
                if(r.extendo.getCurrentPosition()*(-1) > 10)
                {
                    collectAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.COLLECT;
                    r.collect.setPower(-1);
                    CurrentStatus = autoControllerStatus.COLLECT_SAMPLE_DONE;
                }break;
            }

            case COLLECT_SAMPLE_EXTEND_FULL:
            {
                extendoController.CS = org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.EXTENDED_AUTO;
                CurrentStatus = autoControllerStatus.COLLECT_SAMPLE_DONE;
                break;
            }

            case TRANSFER_BEGIN:
            {
                //globals.is_intransfer = true;
                collectAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.DRIVE;
                //outtakeController.CS = outtakeController.outtakeStatus.TRANSFER_LIFT;
                //fourbarController.CS = org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.DRIVE;
                // clawAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.DRIVE;
                r.collect.setPower(-1);
                collectAngle_timer.reset();
                CurrentStatus = autoControllerStatus.TRANSFER_EXTENDO;
                break;
            }

            case TRANSFER_EXTENDO:
            {
                if(collectAngle_timer.seconds()>0.2)
                {
                    extendoController.CS = org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.RETRACTED;
                    CurrentStatus = autoControllerStatus.TRANSFER_SPIT;
                }
                break;
            }

            case TRANSFER_SPIT:
            {
                if(r.extendo.getCurrentPosition()*(-1) < 100)
                {
                    r.collect.setPower(0);
                    claw_timer.reset();
                    CurrentStatus = autoControllerStatus.TRANSFER_CLAW;
                }
                break;
            }

            case TRANSFER_CLAW:
            {
                if(r.extendo.getCurrentPosition()*(-1) < 30 || claw_timer.seconds() > 1.1)
                {
                    claw_timer.reset();
                    r.collect.setPower(0);
                    CurrentStatus = autoControllerStatus.TRANSFER_CLAW_CLOSE;
                }
                break;
            }

            case TRANSFER_CLAW_CLOSE:
            {
                if (claw_timer.seconds() > 0.15)
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
                    if ( globals.is_transfer_summersible == false) CurrentStatus = autoControllerStatus.TRANSFER_DONE;
                        else
                            CurrentStatus = autoControllerStatus.SAMPLE_START;
                }
                break;
            }

            case COLLECT_SUMMERSIBLE_SHORT:
            {
                extendoController.CS = org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.SHORT;
                //collectAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.SWEEP;
                extendoTimer.reset();
                CurrentStatus = autoControllerStatus.COLLECT_SUMMERSIBLE_DONE;
                break;
            }
            case SUMMERSIBLE_RETRACT:
            {
                collectAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.DRIVE;
                extendoController.CS = org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.EXTENDED_AUTO_SUMMERSIBLE;
                break;
            }
            case COLLECT_SUMMERSIBLE_EXTEND_SHORT:
            {
                collectAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.COLLECT;
                extendoController.CS = org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.SHORT;
                r.collect.setPower(-1);
                extendoTimer.reset();
                CurrentStatus = autoControllerStatus.COLLECT_SUMMERSIBLE_EXTEND_LONG;
                break;
            }
            case COLLECT_SUMMERSIBLE_EXTEND_LONG:
            {
                if (extendoTimer.seconds() > 0.2)
                {
                    extendoController.CS = org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.EXTENDED;
                    CurrentStatus = autoControllerStatus.COLLECT_SUMMERSIBLE_DONE;
                }
                break;
            }
            case RETRACT_SHORT:
            {
                collectAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.collectAngleController.collectAngleStatus.SWEEP;
                extendoController.CS = org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.SHORT;
                break;
            }
        }

        //todo switch between cs2 and cs1

        switch (CurrentStatus2)
        {

            case OUTTAKE_DOWN:
            {
                org.firstinspires.ftc.teamcode.system_controllers.liftController.CS = org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.DOWN;
                CurrentStatus2 = autoControllerStatus.OUTTAKE_DOWN_SYSTEMS;
                //outtakeTimer.reset();
                break;
            }

            case OUTTAKE_DOWN_SYSTEMS:
            {
                if(r.collect.getCurrentPosition() < 15000)
                {
                    clawController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawController.clawStatus.OPENED;
                    clawAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.DRIVE;
                    fourbarController.CS = org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.DRIVE;
                    CurrentStatus2 = autoControllerStatus.OUTTAKE_DOWN_DONE;
                    //fourbarTimer.reset();
                    //CurrentStatus2 = autoControllerStatus.OUTTAKE_DOWN_FOURBAR;
                }
                break;
            }

            case OUTTAKE_DOWN_FOURBAR:
            {
                if(fourbarTimer.seconds() > 0.2)
                {
                    fourbarController.CS = org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.DRIVE;
                    CurrentStatus2 = autoControllerStatus.OUTTAKE_DOWN_DONE;
                }
                break;
            }

            case OPEN_CLAW:
            {
                clawController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawController.clawStatus.OPENED;
                CurrentStatus2 = autoControllerStatus.OPEN_CLAW_DONE;
                break;
            }
            case HANG_LV1:
            {
                org.firstinspires.ftc.teamcode.system_controllers.liftController.CS = org.firstinspires.ftc.teamcode.system_controllers.liftController.liftStatus.HANG_LV1;
                fourbarController.CS = org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.HANG_LV1;
                clawAngleController.CS = org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.HANG_LV1;
                CurrentStatus2 = autoControllerStatus.NOTHING;
                break;
            }
        }
        PreviousStatus = CurrentStatus;
        PreviousStatus2 = CurrentStatus2;
    }
}