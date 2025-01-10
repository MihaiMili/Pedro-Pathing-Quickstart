package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_BEGIN;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_CLAW;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_COLLECT_ANGLE_BACK;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_DONE;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_EXTENDO;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_OUTTAKE;
import static org.firstinspires.ftc.teamcode.system_controllers.transferController.transferStatus.TRANSFER_SPIT;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Globals.globals;
import org.firstinspires.ftc.teamcode.Globals.robotMap;

@Config
public class transferController {

    public enum transferStatus
    {
        INITIALIZE,
        TRANSFER_BEGIN,
        TRANSFER_EXTENDO,
        TRANSFER_SPIT,
        TRANSFER_CLAW,
        TRANSFER_CLAW_CLOSE,

        TRANSFER_OUTTAKE,
        TRANSFER_COLLECT_ANGLE_BACK,
        TRANSFER_DONE
    }

    public transferController()
    {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    ElapsedTime collectAngle_timer = new ElapsedTime();
    ElapsedTime claw_timer = new ElapsedTime();

    ElapsedTime collect_angle_back = new ElapsedTime();
    ElapsedTime outtake_timer = new ElapsedTime();

    globals globals = new globals();

public static double extendo_limit = 0.2;
    public static transferStatus CS = INITIALIZE, PS = INITIALIZE;

    public void update(robotMap r, clawController claw, collectAngleController collectAngle, outtakeController outtake, extendoController extendo)
    {
        if(CS != PS || CS == INITIALIZE || CS == TRANSFER_BEGIN || CS == TRANSFER_EXTENDO || CS == TRANSFER_SPIT || CS == TRANSFER_CLAW || CS == TRANSFER_CLAW_CLOSE || CS == TRANSFER_OUTTAKE  || CS == TRANSFER_DONE || CS == TRANSFER_COLLECT_ANGLE_BACK)
        {
            switch (CS)
            {
                case TRANSFER_BEGIN:
                {
                    globals.is_intransfer = true;
                    collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;
                    outtake.CS = outtakeController.outtakeStatus.TRANSFER_LIFT;
                    r.collect.setPower(-1);
                    collectAngle_timer.reset();
                    CS = TRANSFER_EXTENDO;
                    break;
                }

                case TRANSFER_EXTENDO:
                {
                    if(collectAngle_timer.seconds()>extendo_limit)
                    {
                        extendo.CS = extendoController.extendoStatus.TRANSFER;
                        CS = TRANSFER_SPIT;
                    }
                    break;
                }

                case TRANSFER_SPIT:
                {
                    if(r.extendo.getCurrentPosition() < 5000)
                    {
                        r.collect.setPower(0);
                        claw_timer.reset();
                        CS = TRANSFER_CLAW;
                    }
                    break;
                }

                case TRANSFER_CLAW:
                {
                    if(r.extendo.getCurrentPosition() < 1 && r.collect.getCurrentPosition() < 1000)
                    {
                        claw_timer.reset();
                            r.collect.setPower(0);
                        CS = TRANSFER_CLAW_CLOSE;
                    }
                    break;
                }

                case TRANSFER_CLAW_CLOSE:
                {
                    if (claw_timer.seconds() > 0.5)
                    {
                        claw.CS = clawController.clawStatus.NOT_SO_CLOSED;
                        outtake_timer.reset();
                        CS = TRANSFER_OUTTAKE;
                    }
                    break;
                }

                case TRANSFER_OUTTAKE:
                {
                    if(outtake_timer.seconds() > 0.2)
                    {fourbarController.CS = fourbarController.fourbarStatus.TRANSFER;
                    clawAngleController.CS = clawAngleController.clawAngleStatus.TRANSFER;
                    collectAngle.CS = collectAngleController.collectAngleStatus.TRANSFER;
                    collect_angle_back.reset();
                    CS = TRANSFER_COLLECT_ANGLE_BACK;
                    }
                    break;
                }

                case TRANSFER_COLLECT_ANGLE_BACK:
                {
                    if(collect_angle_back.seconds() > 0.2)
                    {
                        collectAngle.CS= collectAngleController.collectAngleStatus.DRIVE;
                        globals.is_intransfer = false;
                        liftController.CS = liftController.liftStatus.DOWN;
                        extendo.CS = extendoController.extendoStatus.RETRACTED;
                        CS = TRANSFER_DONE;
                    }
                    break;
                }


            }
        }

        PS = CS;

    }

}
