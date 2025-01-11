package org.firstinspires.ftc.teamcode.system_controllers;


import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.EXTENDED;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.EXTENDED_AJUSTABIl1;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.EXTENDED_AJUSTABIl2;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.EXTENDED_AUTO;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.EXTENDED_AUTO_SUMMERSIBLE;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.EXTENDED_SPECIMEN;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.RETRACTED;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.SHORT;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.SHORT_AJUSTABIL1;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.SHORT_AJUSTABIL2;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.SPECIMEN1;
import static org.firstinspires.ftc.teamcode.system_controllers.extendoController.extendoStatus.TRANSFER;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Globals.SimplePIDController;
import org.firstinspires.ftc.teamcode.Globals.globals;
import org.firstinspires.ftc.teamcode.Globals.robotMap;

@Config
public class extendoController {
    public enum extendoStatus {
        INITIALIZE,
        RETRACTED,
        EXTENDED,
        EXTENDED_AJUSTABIl1,
        EXTENDED_AJUSTABIl2,
        EXTENDED_AUTO,
        EXTENDED_AUTO_SUMMERSIBLE,
        SHORT,
        SHORT_AJUSTABIL1,
        SHORT_AJUSTABIL2,
        TRANSFER,
        SPECIMEN1,
        SPECIMEN2,
        SPECIMEN3,

        EXTENDED_SPECIMEN,
        SHORT_EXTEND_SPECIMEN,

    }

    // PID constants for extension
    public static double Kp_extend = 0.011;//0.00008;
    public static double Ki_extend = 0;
    public static double Kd_extend = 0;

    public static double Kp_retract = 0.01; //0.01
    public static double Ki_retract = 0; //0
    public static double Kd_retract = 0; //0

    SimplePIDController extendoPIDExtend;
    SimplePIDController extendoPIDRetract;

    public static double maxSpeed = 1;

    public extendoStatus CS = INITIALIZE, PS = INITIALIZE;

    globals globals = new globals();

    public static double retracted = 0;
    public static double extended = /*30000*/ 420;
    public static double drive = 800;
    public static double transfer = -5;
    public static double short_pose = 200;
    public static double summersible_short = 150;
    public static double specimen1 = 200;
    public static double specimen2 = 200;
    public static double specimen3 = 200;
    public static double extended_auto[]= {440, 440, 440};

    public static double extend_specimen[] = {370, 370,370,350};
    public static double short_extend_specimen = 75;

    public static int extendo_i = 0;
    public static double i_multiplication = 50;

    public SimplePIDController activePID;

    public static double extend_multiply_index = 0;

    public extendoController() {
        extendoPIDExtend = new SimplePIDController(Kp_extend, Ki_extend, Kd_extend);
        extendoPIDRetract = new SimplePIDController(Kp_retract, Ki_retract, Kd_retract);

        extendoPIDExtend.targetValue = retracted;
        extendoPIDExtend.maxOutput = maxSpeed;

        extendoPIDRetract.targetValue = retracted;
        extendoPIDRetract.maxOutput = maxSpeed;
    }

    public void update(robotMap r, int position, double powerCap, double voltage) {
        switch (CS) {
            case INITIALIZE:
                activePID = extendoPIDRetract;
                break;
            case EXTENDED: // Define your conditions
                activePID = extendoPIDExtend;
                break;
            case EXTENDED_AJUSTABIl1: // Define your conditions
                activePID = extendoPIDExtend;
                break;
            case EXTENDED_AJUSTABIl2: // Define your conditions
                activePID = extendoPIDExtend;
                break;
            case RETRACTED:
                activePID = extendoPIDRetract;
                break;
            case SHORT:
                activePID = extendoPIDExtend;
                break;
            case SHORT_AJUSTABIL1:
                activePID = extendoPIDExtend;
                break;
            case SHORT_AJUSTABIL2:
                activePID = extendoPIDExtend;
                break;
            case TRANSFER:
                activePID = extendoPIDRetract;
                break;
            case EXTENDED_AUTO:
                activePID = extendoPIDExtend;
                break;
            case EXTENDED_AUTO_SUMMERSIBLE:
                activePID = extendoPIDExtend;
                break;
            case EXTENDED_SPECIMEN:
                activePID = extendoPIDExtend;
                break;
            case SHORT_EXTEND_SPECIMEN:
                activePID = extendoPIDExtend;
                break;
            default:
                activePID = extendoPIDRetract;
                break;
        }

        double powerColectare = activePID.update(position);
        powerColectare = Math.max(-1,Math.min(powerColectare,1));
        if (activePID.targetValue <= 0 && position <=  1 && (CS == RETRACTED|| CS==TRANSFER) ){
            if (CS == RETRACTED) r.extendo.setPower(0);
                else if (CS == TRANSFER) r.extendo.setPower(-0.4);
        } else
        if(activePID.targetValue > 0 || position > 1)
        {
            r.extendo.setPower(powerColectare);
        }

        if (CS == RETRACTED) {
            activePID.targetValue = retracted;
        }

//        if(CS == EXTENDED)
//        {
//            activePID.targetValue = Math.min(Math.max(extended + org.firstinspires.ftc.teamcode.Globals.globals.ajustabil,100),460);
//        }
//
//        if(CS == SHORT)
//        {
//            activePID.targetValue = Math.min(Math.max(short_pose + org.firstinspires.ftc.teamcode.Globals.globals.ajustabil,100),460);
//        }

        if (CS != PS || CS == EXTENDED || CS == RETRACTED || CS == SHORT || CS == TRANSFER || CS == EXTENDED_SPECIMEN || CS == EXTENDED_AUTO || CS == SPECIMEN1 || CS == extendoStatus.SPECIMEN2 || CS == extendoStatus.SPECIMEN3 || CS == EXTENDED_AUTO_SUMMERSIBLE || CS == EXTENDED_AJUSTABIl1 || CS == EXTENDED_AJUSTABIl2 || CS == SHORT_AJUSTABIL1 || CS == SHORT_AJUSTABIL2) {
            switch (CS) {
                case INITIALIZE: {
                    activePID.targetValue = retracted;
                    activePID.maxOutput = 1;
                    break;
                }

                case EXTENDED: {
                    activePID.targetValue = Math.min(Math.max(extended + org.firstinspires.ftc.teamcode.Globals.globals.ajustabil,100),460);
                    activePID.maxOutput = 1;
                    break;
                }

                case EXTENDED_AJUSTABIl1: {
                    activePID.targetValue = Math.min(460,extended+10*globals.ajustabil);
                    activePID.maxOutput = 1;
                    break;
                }

                case EXTENDED_AJUSTABIl2: {
                    activePID.targetValue = Math.max(340,extended-10*globals.ajustabil);
                    activePID.maxOutput = 1;
                    break;
                }

                case EXTENDED_AUTO: {
                    activePID.targetValue = extended_auto[org.firstinspires.ftc.teamcode.Globals.globals.extendo_auto_i];
                    activePID.maxOutput = 0.7;
                    break;
                }

                case EXTENDED_AUTO_SUMMERSIBLE:{
                    activePID.targetValue = summersible_short;
                    activePID.maxOutput = 0.7;
                    break;
                }
                case SHORT_EXTEND_SPECIMEN:
                {
                    activePID.targetValue = short_extend_specimen;
                    activePID.maxOutput = 1;
                    break;
                }


                case RETRACTED: {
                    activePID.targetValue = retracted;
                    activePID.maxOutput = 1;
                    break;
                }

                case SHORT: {
                    activePID.targetValue = Math.min(Math.max(short_pose + org.firstinspires.ftc.teamcode.Globals.globals.ajustabil,100),460);
                    activePID.maxOutput = 1; activePID.targetValue = Math.min(Math.max(extended + org.firstinspires.ftc.teamcode.Globals.globals.ajustabil,100),460);
                }

                case SHORT_AJUSTABIL1: {
                    activePID.targetValue = Math.min(260,short_pose+10*globals.ajustabil);
                    activePID.maxOutput = 1;
                    break;
                }

                case SHORT_AJUSTABIL2: {
                    activePID.targetValue = Math.max(140,short_pose-10*globals.ajustabil);
                    activePID.maxOutput = 1;
                    break;
                }

                case TRANSFER: {
                    activePID.targetValue = transfer;
                    activePID.maxOutput = 1;
                    break;
                }

                case SPECIMEN1:
                {
                    activePID.targetValue = specimen1;
                    activePID.maxOutput = 0.5;
                    break;
                }

                case SPECIMEN2:
                {
                    activePID.targetValue = specimen2;
                    activePID.maxOutput = 0.5;
                    break;
                }

                case SPECIMEN3:
                {
                    activePID.targetValue = specimen3;
                    activePID.maxOutput = 0.5;
                    break;
                }

                case EXTENDED_SPECIMEN: {
                    activePID.targetValue = extend_specimen[org.firstinspires.ftc.teamcode.Globals.globals.extendo_auto_i];
                    activePID.maxOutput = 1;
                    break;
                }




            }
        }
        PS = CS;
    }

}