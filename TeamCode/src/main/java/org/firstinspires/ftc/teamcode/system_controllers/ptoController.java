package org.firstinspires.ftc.teamcode.system_controllers;

import static org.firstinspires.ftc.teamcode.system_controllers.ptoController.ptoStatus.INITIALIZE;
import static org.firstinspires.ftc.teamcode.system_controllers.ptoController.ptoStatus.OFF;
import static org.firstinspires.ftc.teamcode.system_controllers.ptoController.ptoStatus.ON;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Globals.robotMap;

@Config
public class ptoController {

    public enum ptoStatus
    {
        INITIALIZE,
        ON,
        OFF,
    }

    public ptoController()
    {
        CS = INITIALIZE;
        PS = INITIALIZE;
    }

    public static ptoStatus CS = INITIALIZE, PS = INITIALIZE;

    public static double on = 0.64;
    public static double off = 0.95;

    public void update(robotMap r)
    {
        if(CS != PS || CS == INITIALIZE || CS == ON || CS == OFF)
        {
            switch (CS)
            {
                case INITIALIZE:
                {
                    r.pto.setPosition(off);
                    break;
                }

                case ON:
                {
                    r.pto.setPosition(on);
                    break;
                }

                case OFF:
                {
                    r.pto.setPosition(off);
                    break;
                }

            }
        }

        PS = CS;
    }

}
