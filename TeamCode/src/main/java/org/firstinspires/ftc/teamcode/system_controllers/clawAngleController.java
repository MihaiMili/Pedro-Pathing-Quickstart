package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.COLLECT_SPECIMEN;
import static org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.DRIVE;
import static org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.HANG_LV1;
import static org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.HANG_LV2;
import static org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.MOVE;
import static org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.SAMPlE_SCORE;
import static org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.SPECIMEN;
import static org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.SPECIMEN_SCORE;
import static org.firstinspires.ftc.teamcode.system_controllers.clawAngleController.clawAngleStatus.TRANSFER;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Globals.robotMap;

@Config
public class clawAngleController {

    public enum clawAngleStatus
    {
        INITIALIZE,
        COLLECT_SPECIMEN,
        SAMPlE_SCORE,
        SPECIMEN_SCORE,
        TRANSFER,
        DRIVE,
        SPECIMEN,
        HANG_LV1,
        HANG_LV2,
        MOVE
    }

    public clawAngleController()
    {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static clawAngleStatus CS = INITIALIZE, PS = INITIALIZE;

    public static double collect_specimen = 0.52;
    public static double sample_score = 0.2;
    public static double specimen = 0.35;
    public static double drive = 0.43;
    public static double transfer = 0.67;
    public static double specimen_score = 0.38;//0.5;
    public static double hang_lv1 = 0.45;
    public static double hang_lv2 = 0.45;
    public static double move = 0.3;

    public void update(robotMap r)
    {
        if(PS != CS || CS == INITIALIZE || CS == COLLECT_SPECIMEN || CS == SAMPlE_SCORE || CS == SPECIMEN_SCORE || CS == TRANSFER || CS == SPECIMEN || CS == DRIVE || CS == MOVE || CS == HANG_LV1 || CS == HANG_LV2)

        {
            switch (CS)
            {
                case INITIALIZE:
                {
                    //r.clawAngle.setPosition(transfer);
                    r.clawAngle.setPosition(drive);
                    break;
                }

                case DRIVE:
                {
                    r.clawAngle.setPosition(drive);
                    break;
                }

                case COLLECT_SPECIMEN:
                {
                    r.clawAngle.setPosition(collect_specimen);
                    break;
                }

                case SAMPlE_SCORE:
                {
                    r.clawAngle.setPosition(sample_score);
                    break;
                }

                case SPECIMEN_SCORE:
                {
                    r.clawAngle.setPosition(specimen_score);
                    break;
                }

                case TRANSFER:
                {
                    r.clawAngle.setPosition(transfer);
                    break;
                }

                case SPECIMEN:
                {
                    r.clawAngle.setPosition(specimen);
                    break;
                }

                case HANG_LV1:
                {
                    r.clawAngle.setPosition(hang_lv1);
                    break;
                }

                case HANG_LV2:
                {
                    r.clawAngle.setPosition(hang_lv2);
                    break;
                }

                case MOVE:
                {
                    r.clawAngle.setPosition(move);
                    break;
                }
            }
        }

        PS = CS;
    }

}
