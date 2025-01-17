package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.COLLECT_SPECIMEN;
import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.COLLECT_SPECIMEN_INTER;
import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.DRIVE;
import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.HANG_LV1;
import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.HANG_LV2;
import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.SAMPLE;
import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.SCORE_SPECIMEN;
import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.SPECIMEN;
import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.TRANSFER;
import static org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.UP_SPECIMEN;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Globals.robotMap;

@Config
public class fourbarController {

    public enum fourbarStatus {
        SCORE_SPECIMEN,
        COLLECT_SPECIMEN,
        INITIALIZE,
        SAMPLE,
        SPECIMEN,
        TRANSFER,
        DRIVE,
        INTER,
        COLLECT_SPECIMEN_INTER,
        HANG_LV1,
        HANG_LV2,
        UP_SPECIMEN,
    }

    public fourbarController() {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static fourbarStatus CS = INITIALIZE, PS = INITIALIZE;

    public static double score_specimen = 0.72;//0.9;
    public static double collect = 0.1;
    public static double sample = 0.42;
    public static double specimen_up = 0.7;

    public static double specimen = 0.87;
    public static double drive = 1;
    public static double transfer = 0.8;
    public static double inter  = 0.6;
    public static double collect_specimen_inter = 0.4;
    public static double hang_lv1 = 0.23 ;
    public static double hang_lv2 = 0.23;

    public void update(robotMap r)
    {
        if (PS != CS || CS == INITIALIZE || CS == DRIVE || CS == COLLECT_SPECIMEN  || CS == SCORE_SPECIMEN  || CS == TRANSFER  || CS == SPECIMEN || CS == SAMPLE || CS == COLLECT_SPECIMEN_INTER || CS == HANG_LV1 || CS == HANG_LV2 || CS == UP_SPECIMEN)

        {

            switch (CS) {
                case INITIALIZE: {
                    //r.fourbarLeft.setPosition(transfer);
                    //r.fourbarRight.setPosition(transfer);
                    r.fourbarLeft.setPosition(drive);
                    r.fourbarRight.setPosition(drive);
                    break;
                }

                case UP_SPECIMEN:
                {
                    r.fourbarLeft.setPosition(specimen_up);
                    r.fourbarRight.setPosition(specimen_up);
                    break;
                }

                case COLLECT_SPECIMEN: {
                    r.fourbarLeft.setPosition(collect);
                    r.fourbarRight.setPosition(collect);
                    break;
                }

                case DRIVE:
                {
                    r.fourbarLeft.setPosition(drive);
                    r.fourbarRight.setPosition(drive);
                    break;
                }

                case SCORE_SPECIMEN: {
                    r.fourbarLeft.setPosition(score_specimen);
                    r.fourbarRight.setPosition(score_specimen);
                    break;
                }

                case SAMPLE: {
                    r.fourbarLeft.setPosition(sample);
                    r.fourbarRight.setPosition(sample);
                    break;
                }

                case SPECIMEN: {
                    r.fourbarLeft.setPosition(specimen);
                    r.fourbarRight.setPosition(specimen);
                    break;
                }

                case TRANSFER: {
                    r.fourbarLeft.setPosition(transfer);
                    r.fourbarRight.setPosition(transfer);
                    break;
                }

                case INTER:
                {
                    r.fourbarLeft.setPosition(inter);
                    r.fourbarRight.setPosition(inter);
                    break;
                }

                case COLLECT_SPECIMEN_INTER:
                {
                    r.fourbarLeft.setPosition(collect_specimen_inter);
                    r.fourbarRight.setPosition(collect_specimen_inter);
                    break;
                }

                case HANG_LV1:
                {
                    r.fourbarLeft.setPosition(hang_lv1);
                    r.fourbarRight.setPosition(hang_lv1);
                    break;
                }

                case HANG_LV2:
                {
                    r.fourbarLeft.setPosition(hang_lv2);
                    r.fourbarRight.setPosition(hang_lv2);
                    break;
                }
            }
        }
        PS = CS;
    }

}
