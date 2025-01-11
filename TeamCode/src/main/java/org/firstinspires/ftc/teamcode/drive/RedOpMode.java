package org.firstinspires.ftc.teamcode.drive;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Globals.globals;
import org.firstinspires.ftc.teamcode.Globals.robotMap;
import org.firstinspires.ftc.teamcode.system_controllers.clawAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.clawController;
//import org.firstinspires.ftc.teamcode.system_controllers.climbController;
import org.firstinspires.ftc.teamcode.system_controllers.collectAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.extendoController;
import org.firstinspires.ftc.teamcode.system_controllers.fourbarController;
import org.firstinspires.ftc.teamcode.system_controllers.liftController;
import org.firstinspires.ftc.teamcode.system_controllers.outtakeController;
import org.firstinspires.ftc.teamcode.system_controllers.ptoController;
//import org.firstinspires.ftc.teamcode.system_controllers.ruletaController;
import org.firstinspires.ftc.teamcode.system_controllers.transferController;

import java.util.List;
import java.util.Objects;

@TeleOp(name="Red", group="OpMode")
public class RedOpMode extends LinearOpMode
{
    public static double  PrecisionDenominatorTranslational = 1, PrecisionDenominatorAngle = 1;
    public boolean can_transfer = true;
    public boolean has_sample = false;
    public boolean can_outtake = true;
    public String recognition = "edoarepulamicasieumare";
    public boolean can_init = false;
    public boolean can_hang = false;
    double ajustabil =0;

    public void robotCentricDrive(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack, double  SpeedLimit, boolean StrafesOn , double LeftTrigger, double RightTrigger, boolean pto_ON)
    {
        double y = -gamepad1.right_stick_y; // Remember, this is reversed!
        double x = gamepad1.right_stick_x;
        if (!StrafesOn)
        {
            x=0;
        }

        double rx = gamepad1.left_stick_x*1 - LeftTrigger + RightTrigger;

        rx*=PrecisionDenominatorAngle;
        x/=PrecisionDenominatorTranslational;
        y/=PrecisionDenominatorTranslational;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftPower = Clip(frontLeftPower,SpeedLimit);
        backLeftPower = Clip(backLeftPower,SpeedLimit);
        frontRightPower = Clip(frontRightPower,SpeedLimit);
        backRightPower = Clip(backRightPower,SpeedLimit);

        if(frontRightPower > 0.1 || frontLeftPower > 0.1 || backLeftPower > 0.1 || backRightPower > 0.1) can_init = true;

        if(pto_ON)
        {
            if(gamepad1.right_trigger > 0)
            {
                leftBack.setPower(-1);
                rightBack.setPower(-1);
            }
            else
            {
                leftBack.setPower(backLeftPower);
                rightBack.setPower(backRightPower);
            }
            leftFront.setPower(0);
            rightFront.setPower(0);

        } else
        {
            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);
        }


    }
    public String Retrun_Color(ColorSensor colorSensor)
    {
        int red_value = colorSensor.red();
        int  green_value = colorSensor.green();
        int blue_value = colorSensor.blue();

        if(red_value > green_value && red_value > blue_value && red_value > 300)
        {
            return "red";

        }
        else if(green_value > blue_value && green_value > 500)
        {
            return "yellow";
        }
        else if(blue_value > 500)
        {

            return "blue";
        }
        else {return  "nothing";}
    }

    double Clip(double Speed, double lim)
    {
        return Math.max(Math.min(Speed,lim), -lim);
    }




    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode()
    {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        robotMap r = new robotMap(hardwareMap);

        /**
         * SYSTEM CONTROLLERS
         */

        clawController claw = new clawController();
        clawAngleController clawAngle = new clawAngleController();
        collectAngleController collectAngle = new collectAngleController();
        extendoController extendo = new extendoController();
        fourbarController fourbar = new fourbarController();
        liftController lift = new liftController();
        outtakeController outtake = new outtakeController();
        ptoController pto = new ptoController();
        transferController transfer = new transferController();
        //ruletaController ruleta = new ruletaController();
        //climbController climb = new climbController();

        globals globals = new globals();


        /**
         * INITS
         */

        double voltage;
        double loopTime = 0;
        ElapsedTime timer_check = new ElapsedTime();
        ElapsedTime timer_specimen = new ElapsedTime();
        ElapsedTime timer_loop = new ElapsedTime();
        ElapsedTime timer_hang = new ElapsedTime();
        ElapsedTime timer_ajustabil = new ElapsedTime();
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage = batteryVoltageSensor.getVoltage();



//        /**
//         * UPDATES
//         */
//
//        claw.update(r);
//        clawAngle.update(r);
//        collectAngle.update(r);
//        extendo.update(r,0,0.5,voltage);
//        fourbar.update(r);
//        lift.update(r,0,voltage);
//        pto.update(r);
//        transfer.update(r,claw,collectAngle,outtake,extendo);
//        outtake.update(r,claw,lift,fourbar,clawAngle,transfer);

        /**
         * OTHERS INITS
         */

        boolean StrafesOn = true;
        boolean pto_ON = false;
        double SpeedLimit = 1;
        int okSpecimenHigh = 0;
        int okSpecimenLow = 0;
        can_outtake = true;


        boolean haveSample=false;
        org.firstinspires.ftc.teamcode.Globals.globals.is_scoring_specimen = false;
        org.firstinspires.ftc.teamcode.Globals.globals.is_intransfer = false;

        Gamepad.RumbleEffect effectCollect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 1000)
                .addStep(0.0, 0.0, 50)
                .addStep(1.0, 1.0, 100)
                .build();

        double collectPower = 0;
        //double liftPower = 0;
        int position_lift;
        int position_extendo;
        org.firstinspires.ftc.teamcode.Globals.globals.ajustabil = 0;
        int red_value;
        int green_value;
        int blue_value;
        String color_value = "nothing";


        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested())
        {
            /**
             * INITS
             */

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            MotorConfigurationType motorConfigurationType = r.leftBack.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            r.leftBack.setMotorType(motorConfigurationType);

            motorConfigurationType = r.rightBack.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            r.rightBack.setMotorType(motorConfigurationType);

            motorConfigurationType = r.rightFront.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            r.rightFront.setMotorType(motorConfigurationType);

            motorConfigurationType = r.leftFront.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            r.leftFront.setMotorType(motorConfigurationType);

            motorConfigurationType = r.extendo.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            r.extendo.setMotorType(motorConfigurationType);

            motorConfigurationType = r.lift.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            r.lift.setMotorType(motorConfigurationType);

            position_lift = r.collect.getCurrentPosition();
            position_extendo = r.extendo.getCurrentPosition()*(-1);

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);


            robotCentricDrive(r.leftFront, r.leftBack, r.rightFront, r.rightBack, SpeedLimit , StrafesOn , 0,0, pto_ON);

            /**
             * BUTON INIT
             */

            if(!previousGamepad1.dpad_up && currentGamepad1.dpad_up)
            {
                StrafesOn = !StrafesOn;
            }

            if(!previousGamepad2.touchpad && currentGamepad2.touchpad)
            {
                can_hang = !can_hang;
            }

            if(position_lift>3000 || position_extendo>40 || outtake.CS == outtakeController.outtakeStatus.COLLECT_SPECIMEN || outtake.CS == outtakeController.outtakeStatus.COLLECT_SPECIMEN_2 || outtake.CS == outtakeController.outtakeStatus.COLLECT_SPECIMEN_3 || outtake.CS == outtakeController.outtakeStatus.COLLECT_SPECIMEN_DONE)
            {
                PrecisionDenominatorAngle = 0.4;
            }
            else PrecisionDenominatorAngle = 1;
            /**
             * COLLECT
             */

            collectPower = gamepad2.left_trigger*0.75 - gamepad2.right_trigger;

//            if (timer_check.seconds()<0.2)
//            {
//                if (can_transfer == true && globals.is_intransfer == false)
//                {
//                    transfer.CS = transferController.transferStatus.TRANSFER_BEGIN;
//                    can_transfer = false;
//                }
//            }


            if(can_hang == true)
            {
                if(!previousGamepad2.square && currentGamepad2.square)
                {
                    //if(outtake.CS != outtakeController.outtakeStatus.HANG_LV2) outtake.CS = outtakeController.outtakeStatus.HANG_LV2;
                    if(lift.CS != liftController.liftStatus.HANG_LV2) lift.CS = liftController.liftStatus.HANG_LV2;
                    else lift.CS = liftController.liftStatus.HANG_LIL_DOWN;
//                    else
//                    {
//                        pto_ON = true;
//                        pto.CS = ptoController.ptoStatus.ON;
//                        //timer_hang.reset();
//                    }
                }

//                if(timer_hang.seconds() > 0.5 && pto_ON)
//                {
//                    outtake.CS = outtakeController.outtakeStatus.HANG_DOWN;
//                }

                if(!previousGamepad2.cross && currentGamepad2.cross)
                {
                    pto_ON=!pto_ON;
                }

                if(pto_ON) {pto.CS = ptoController.ptoStatus.ON;r.lift.setPower(0);}
                else pto.CS = ptoController.ptoStatus.OFF;

//                liftPower = gamepad1.right_trigger - gamepad1.left_trigger;
//
//                if(liftPower>0) r.lift.setPower(1);
//                else if(liftPower<0) r.lift.setPower(-1);
//                else r.lift.setPower(0);

                if(-collectPower > 0) r.ruleta.setPower(1);
                else if(-collectPower <0) r.ruleta.setPower(-1);
                else r.ruleta.setPower(0);
            }
            else
            {
                if(org.firstinspires.ftc.teamcode.Globals.globals.is_intransfer == false)
                {
                    r.collect.setPower(collectPower);
                }

                if(collectAngle.CS == collectAngleController.collectAngleStatus.COLLECT && collectPower < 0)
                {
                    if (timer_loop.seconds() > 0.05)
                    {
                        recognition = Retrun_Color(r.color_sensor);
                        timer_loop.reset();
                    }
                    if(Objects.equals(recognition, "nothing"))
                    {
                        has_sample = false;
                    }
                    else
                    {
                        has_sample = true;
                    }
                    if(Objects.equals(recognition, "blue"))
                    {
                        gamepad1.runRumbleEffect(effectCollect);
                        gamepad2.runRumbleEffect(effectCollect);
                        can_transfer = false;
                    }
                    else {
                        can_transfer = true;

                    }
//                if(Objects.equals(Retrun_Color(r.color_sensor), "red") || Objects.equals(Retrun_Color(r.color_sensor), "yellow"))
//                {
//
//                }

                }

//            if(r.color_sensor.getDistance(DistanceUnit.MM)<10)
//            {
//                gamepad1.runRumbleEffect(effectCollect);
//                gamepad2.runRumbleEffect(effectCollect);
//            }

                /**
                 * OUTTAKE
                 */

//            if(position_extendo > 3000) can_outtake = false;
//            else can_outtake = true;

                if(globals.is_intransfer == false)
                {

                    /**
                     * SPECIMEN SCORE
                     */

                    if (!previousGamepad2.circle && currentGamepad2.circle) {
                        if (outtake.CS == outtakeController.outtakeStatus.SPECIMEN_HIGH ) {
                            outtake.CS = outtakeController.outtakeStatus.SPECIMEN_LOW;
                        } else if (outtake.CS == outtakeController.outtakeStatus.COLLECT_SPECIMEN_DONE) {
                            outtake.CS = outtakeController.outtakeStatus.SPECIMEN_LOW;
                        } else if (outtake.CS == outtakeController.outtakeStatus.SPECIMEN_LOW) {
                            outtake.CS = outtakeController.outtakeStatus.SCORE_SPECIMEN;
                        } else if (outtake.CS == outtakeController.outtakeStatus.SAMPLE_LOW) {
                            outtake.CS = outtakeController.outtakeStatus.TRANSFER_LIFT;
                        } else {
                            outtake.CS = outtakeController.outtakeStatus.SAMPLE_LOW;
                        }
                    }
                    if (!previousGamepad2.dpad_up && currentGamepad2.dpad_up) {
                        if (clawAngle.CS != clawAngleController.clawAngleStatus.MOVE)
                            clawAngle.CS = clawAngleController.clawAngleStatus.MOVE;
                        else clawAngle.CS = clawAngleController.clawAngleStatus.SPECIMEN;
                    }
                    if (!previousGamepad2.cross && currentGamepad2.cross) {
                        if (outtake.CS == outtakeController.outtakeStatus.SPECIMEN_LOW) {
                            outtake.CS = outtakeController.outtakeStatus.SPECIMEN_HIGH;
                        } else if (outtake.CS == outtakeController.outtakeStatus.COLLECT_SPECIMEN_DONE) {
                            outtake.CS = outtakeController.outtakeStatus.SPECIMEN_HIGH;
                            clawAngle.CS = clawAngleController.clawAngleStatus.SPECIMEN;
                        } else if (outtake.CS == outtakeController.outtakeStatus.SPECIMEN_HIGH) {
                            outtake.CS = outtakeController.outtakeStatus.SCORE_SPECIMEN;
                            timer_specimen.reset();
                        } else if (outtake.CS == outtakeController.outtakeStatus.SAMPLE_HIGH || outtake.CS == outtakeController.outtakeStatus.SAMPLE_HIGH_FOURBAR) {
                            outtake.CS = outtakeController.outtakeStatus.TRANSFER_LIFT;
                        } else {
                            outtake.CS = outtakeController.outtakeStatus.SAMPLE_HIGH;
                        }
                    }


//                    if (!previousGamepad2.square && currentGamepad2.square) {
//                        if(climb.CS != climbController.climbStatus.HANG_UP) climb.CS = climbController.climbStatus.HANG_UP;
//                        else climb.CS = climbController.climbStatus.HANG_BEGIN;
//                    }

                    /**
                     * SPECIMEN COLLECT
                     */

                    if (!previousGamepad2.triangle && currentGamepad2.triangle && timer_specimen.seconds()>0.5) {
                        if (outtake.CS == outtakeController.outtakeStatus.COLLECT_SPECIMEN || outtake.CS == outtakeController.outtakeStatus.COLLECT_SPECIMEN_2 || outtake.CS == outtakeController.outtakeStatus.COLLECT_SPECIMEN_3 || outtake.CS == outtakeController.outtakeStatus.COLLECT_SPECIMEN_DONE) {
                            outtake.CS = outtakeController.outtakeStatus.TRANSFER_LIFT;
                        } else outtake.CS = outtakeController.outtakeStatus.COLLECT_SPECIMEN;
                    }


                    if(!previousGamepad2.dpad_right && currentGamepad2.dpad_right && org.firstinspires.ftc.teamcode.Globals.globals.is_intransfer == false && extendo.CS != extendoController.extendoStatus.RETRACTED)
                    {
                        if(position_extendo > 50)
                        {if(collectAngle.CS != collectAngleController.collectAngleStatus.COLLECT)
                        {
                            collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
                        } else
                        {
                            collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;
                        }}
                    }

                    if(!previousGamepad2.dpad_down && currentGamepad2.dpad_down)
                    {
                        if(collectAngle.CS != collectAngleController.collectAngleStatus.SWEEP) collectAngle.CS = collectAngleController.collectAngleStatus.SWEEP;
                        else collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;
                    }

                    /**
                     * SAMPLE
                     */


                }
                if(org.firstinspires.ftc.teamcode.Globals.globals.is_intransfer == false)
                {if(!previousGamepad2.right_bumper && currentGamepad2.right_bumper)
                {
                    timer_check.reset();
                    if(outtake.CS == outtakeController.outtakeStatus.COLLECT_SPECIMEN_DONE )
                    {
                        if(claw.CS != clawController.clawStatus.OPENED)
                        {
                            claw.CS = clawController.clawStatus.OPENED;
                        }
                        else {claw.CS = clawController.clawStatus.CLOSED;
                            gamepad1.runRumbleEffect(effectCollect);
                            gamepad2.runRumbleEffect(effectCollect);}
                    }
                    else {if(claw.CS != clawController.clawStatus.NOT_SO_CLOSED)
                    {
                        claw.CS = clawController.clawStatus.NOT_SO_CLOSED;
                    }
                    else {claw.CS = clawController.clawStatus.OPENED;
                        gamepad1.runRumbleEffect(effectCollect);
                        gamepad2.runRumbleEffect(effectCollect);}}
                }}

                /**
                 * EXTENDO
                 */

                if(!previousGamepad1.right_bumper && currentGamepad1.right_bumper && org.firstinspires.ftc.teamcode.Globals.globals.is_intransfer == false)
                {

                    if(extendo.CS != extendoController.extendoStatus.EXTENDED)
                    {
                        extendo.CS = extendoController.extendoStatus.EXTENDED;
                        ajustabil = 0;
                    }
                    else
                    {
                        extendo.CS = extendoController.extendoStatus.RETRACTED;
                        ajustabil = 0;
                      //  org.firstinspires.ftc.teamcode.Globals.globals.ajustabil = 0;
                        collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;

                    }
                }

//                if(gamepad1.right_trigger > 0 && timer_ajustabil.seconds() > 0.1)
//                {
//                    timer_ajustabil.reset();
//                    globals.ajustabil++;
//                    if(position_extendo > 300)
//                    {
//                         extendo.CS = extendoController.extendoStatus.EXTENDED_AJUSTABIl1;
//                    }
//                    else extendo.CS = extendoController.extendoStatus.SHORT_AJUSTABIL1;
//                }
//
//                if(gamepad1.left_trigger > 0 && timer_ajustabil.seconds() > 0.1)
//                {
//                    timer_ajustabil.reset();
//                    globals.ajustabil++;
//                    if(position_extendo > 300)
//                    {
//                        extendo.CS = extendoController.extendoStatus.EXTENDED_AJUSTABIl2;
//                    }
//                    else extendo.CS = extendoController.extendoStatus.SHORT_AJUSTABIL2;
//                }



                if(has_sample && can_transfer && org.firstinspires.ftc.teamcode.Globals.globals.is_intransfer == false)
                {
                    transfer.CS = transferController.transferStatus.TRANSFER_BEGIN;
                    has_sample = false;
                    can_transfer= false;
                }

                if(!previousGamepad1.left_bumper && currentGamepad1.left_bumper && org.firstinspires.ftc.teamcode.Globals.globals.is_intransfer == false)
                {
                    if(extendo.CS != extendoController.extendoStatus.SHORT)
                    {
                        extendo.CS = extendoController.extendoStatus.SHORT;
                        ajustabil = 0;
                    }
                    else
                    {
                        extendo.CS = extendoController.extendoStatus.RETRACTED;
                        collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;
                        ajustabil = 0;
                    }
                }
            }

//           ajustabil = gamepad1.right_trigger - gamepad1.left_trigger;
//
//            if(ajustabil > 0 && timer_ajustabil.seconds() > 0.007)
//            {
//                timer_ajustabil.reset();
//                org.firstinspires.ftc.teamcode.Globals.globals.ajustabil = Math.min(org.firstinspires.ftc.teamcode.Globals.globals.ajustabil + 5,200);
//            }
//
//            if(ajustabil < 0 && timer_ajustabil.seconds() > 0.007)
//            {
//                timer_ajustabil.reset();
//                org.firstinspires.ftc.teamcode.Globals.globals.ajustabil = Math.max(org.firstinspires.ftc.teamcode.Globals.globals.ajustabil - 5, -200);
//            }



            /**
             * UPDATES
             */


            if(can_init) {
                clawAngle.update(r);
                claw.update(r);
                collectAngle.update(r);
                extendo.update(r, position_extendo, 1, voltage);
                fourbar.update(r);
                if(pto_ON == false) {lift.update(r, position_lift, voltage);
                outtake.update(r, claw, lift, fourbar, clawAngle, transfer);}
                transfer.update(r, claw, collectAngle, outtake, extendo);
                pto.update(r);
                //ruleta.update(r);
                //climb.update(r,outtake,pto,ruleta);
            }




            double loop = System.nanoTime();

            telemetry.addData("hz", 1000000000 / (loop - loopTime));
//            telemetry.addData("collectPwr",collectPower);
//            telemetry.addData("extendopos", position_extendo);
//            telemetry.addData("liftpos",position_lift);
//            telemetry.addData("transfer", transfer.CS);
//            telemetry.addData("claw", claw.CS);
//            telemetry.addData("extendo", extendo.CS);
//            telemetry.addData("outake", outtake.CS);
//            telemetry.addData("fourbar", fourbar.CS);
//            telemetry.addData("clawangle", clawAngle.CS);
//            telemetry.addData("clawAnglePos",r.clawAngle.getPosition());
//            telemetry.addData("collectangle", collectAngle.CS);
//            telemetry.addData("denominator",PrecisionDenominatorAngle);
           telemetry.addData("hassample", has_sample);
//            telemetry.addData("cantransfer", can_transfer);
//            telemetry.addData("can_hang",can_hang);
//            telemetry.addData("isintransfer", org.firstinspires.ftc.teamcode.Globals.globals.is_intransfer);
//            telemetry.addData("ajustabil", org.firstinspires.ftc.teamcode.Globals.globals.ajustabil);
//            try
//            {telemetry.addData("target value",extendo.activePID.targetValue);} catch (Exception e) {}


            //telemetry.addData("color dist", r.color_sensor.getDistance(DistanceUnit.MM));
//            telemetry.addData("red", r.color_sensor.red());
//            telemetry.addData("green", r.color_sensor.green());
//            telemetry.addData("blue", r.color_sensor.blue());
            telemetry.addData("color_value",color_value);

            loopTime = loop;
            telemetry.update();


        }
    }
}