package org.firstinspires.ftc.teamcode.drive;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Globals.robotMap;
import org.firstinspires.ftc.teamcode.system_controllers.clawAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.clawController;
import org.firstinspires.ftc.teamcode.system_controllers.collectAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.extendoController;
import org.firstinspires.ftc.teamcode.system_controllers.fourbarController;
import org.firstinspires.ftc.teamcode.system_controllers.liftController;
import org.firstinspires.ftc.teamcode.system_controllers.outtakeController;

import java.util.List;


@TeleOp(name="testTeleOP", group="OpMode")
public class TestOpMode extends LinearOpMode {


    public static double  PrecisionDenominatorTranslational = 1, PrecisionDenominatorAngle = 1;

    double power_lift_test;
    public void robotCentricDrive(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack, double  SpeedLimit, boolean StrafesOn , double LeftTrigger, double RightTrigger, boolean pto_ON)
    {
        double y = -gamepad1.right_stick_y; // Remember, this is reversed!
        double x = gamepad1.right_stick_x;
        if (StrafesOn == false)
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

        if(pto_ON == true)
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

//        clawController claw = new clawController();
//        clawAngleController clawAngle = new clawAngleController();
//       collectAngleController collectAngle = new collectAngleController();
//        extendoController extendo = new extendoController();
//        fourbarController fourbar = new fourbarController();
//        outtakeController outtake = new outtakeController();
//       // testfsm testfsm = new testfsm();
//        liftController lift = new liftController();
////        outtakeController outtake = new outtakeController();
////        ptoController pto = new ptoController();
////        transferController transfer = new transferController();
////
////        globals globals = new globals();
//

        /**
         * INITS
         */

        double voltage;
        double loopTime = 0;
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage = batteryVoltageSensor.getVoltage();



        /**
         * UPDATES
         */

//        claw.update(r);
//       clawAngle.update(r);
//       collectAngle.update(r);
//        //extendo.update(r,0,0.5,voltage);
//        fourbar.update(r);
      //  testfsm.update(fourbar, clawAngle);
        //lift.update(r,0,voltage);

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

        Gamepad.RumbleEffect effectCollect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 100)
                .addStep(0.0, 0.0, 50)
                .addStep(1.0, 1.0, 100)
                .build();

        double collectPower = 0;
        int position_lift;
        int position_extendo;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        while (opModeIsActive() && !isStopRequested()) {
            /**
             * INITS
             */

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

//            MotorConfigurationType motorConfigurationType = r.leftBack.getMotorType().clone();
//            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
//            r.leftBack.setMotorType(motorConfigurationType);
//
//            motorConfigurationType = r.rightBack.getMotorType().clone();
//            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
//            r.rightBack.setMotorType(motorConfigurationType);
//
//            motorConfigurationType = r.rightFront.getMotorType().clone();
//            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
//            r.rightFront.setMotorType(motorConfigurationType);
//
//            motorConfigurationType = r.leftFront.getMotorType().clone();
//            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
//            r.leftFront.setMotorType(motorConfigurationType);
//
//            motorConfigurationType = r.extendo.getMotorType().clone();
//            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
//            r.extendo.setMotorType(motorConfigurationType);
//
//            motorConfigurationType = r.lift.getMotorType().clone();
//            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
//            r.lift.setMotorType(motorConfigurationType);
//
//            position_lift = r.collect.getCurrentPosition();
//            position_extendo = r.extendo.getCurrentPosition();

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);


            //power_lift_test = currentGamepad1.right_trigger - currentGamepad1.left_trigger;
            //r.extendo.setPower(power_  _test);

            //robotCentricDrive(r.leftFront, r.leftBack, r.rightFront, r.rightBack, SpeedLimit, StrafesOn, 0, 0, pto_ON);
            //r.extendo.setPower(currentGamepad1.right_trigger-currentGamepad1.left_trigger);
            /**
             * COLLECT
             */



            collectPower = gamepad1.right_trigger - gamepad1.left_trigger;
            //r.collect.setPower(collectPower);

            if(collectPower > 0) r.ruleta.setPower(1);
            else if(collectPower <0) r.ruleta.setPower(-1);
            else r.ruleta.setPower(0);

//            if (r.color_sensor.getDistance(DistanceUnit.MM) < 10) {
//                gamepad1.runRumbleEffect(effectCollect);
//
//            }

            /**
             * BUTON INIT
             */

//            if (!previousGamepad1.touchpad && currentGamepad1.touchpad) {
//                clawAngle.CS = clawAngleController.clawAngleStatus.DRIVE;
//                claw.CS = clawController.clawStatus.OPENED;
//                collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;
//                extendo.CS = extendoController.extendoStatus.RETRACTED;
//                fourbar.CS = fourbarController.fourbarStatus.DRIVE;
//                lift.CS = liftController.liftStatus.DOWN;
//                outtake.CS = outtakeController.outtakeStatus.DRIVE;
//            }

            /**
             * EXTENDO
             **/

//                if (gamepad1.right_bumper) extendo.CS = extendoController.extendoStatus.SHORT;
//                if (gamepad1.left_bumper) extendo.CS = extendoController.extendoStatus.EXTENDED;
//            if(gamepad1.right_trigger>0)
//            {
//                r.extendo.setPower(0.5);
//            }
//            else r.extendo.setPower(0);
//
//            if(gamepad1.left_trigger>0)
//            {
//                r.extendo.setPower(-0.5);
//            }
//            else r.extendo.setPower(0);

            /**
             * LIFT
             */

//            if(gamepad2.right_stick_y>0)
//            {
//                r.lift.setPower(0.5);
//            }
//            else r.lift.setPower(0);
//
//            if(-gamepad1.right_stick_y>0)
//            {
//                r.lift.setPower(-0.5);
//            }
//            else r.lift.setPower(0);

            /**
             * CLAW
             */
//            if (!previousGamepad2.left_bumper && currentGamepad2.left_bumper) {
//                if (clawAngle.CS != clawAngleController.clawAngleStatus.MOVE)
//                    clawAngle.CS = clawAngleController.clawAngleStatus.MOVE;
//                else clawAngle.CS = clawAngleController.clawAngleStatus.SPECIMEN;
//            }
//            if (!previousGamepad2.right_bumper && currentGamepad2.right_bumper) {
//                if (claw.CS != clawController.clawStatus.OPENED) {
//                    claw.CS = clawController.clawStatus.OPENED;
//                } else claw.CS = clawController.clawStatus.CLOSED;
//            }

            /**
             * COLLECT ANGLE
             */

//            if(!previousGamepad2.square && currentGamepad2.square)
//            {
//                if(collectAngle.CS != collectAngleController.collectAngleStatus.DRIVE)
//                {
//                    collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;
//                }
//                else collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
//            }

            /**
             * FOURBAR
             */

//                        if(!previousGamepad2.cross && currentGamepad2.cross)
//                        {
//                           if(fourbar.CS != fourbarController.fourbarStatus.SCORE_SPECIMEN)
//                           {
//                               fourbar.CS = fourbarController.fourbarStatus.SCORE_SPECIMEN;
//                               clawAngle.CS = clawAngleController.clawAngleStatus.SPECIMEN_SCORE;
//                           }
//                           else {
//                               fourbar.CS = fourbarController.fourbarStatus.SAMPLE;
//                               clawAngle.CS = clawAngleController.clawAngleStatus.SAMPlE_SCORE;
//                           }
//                        }
//
//
//            /**
//             * TRANSFER
//             */
//
//            if(!previousGamepad2.dpad_down && currentGamepad2.dpad_down)
//            {
//                if(fourbar.CS != fourbarController.fourbarStatus.SPECIMEN) fourbar.CS = fourbarController.fourbarStatus.SPECIMEN;
//                else fourbar.CS = fourbarController.fourbarStatus.TRANSFER;
//            }
//
//            if(!previousGamepad2.dpad_up && currentGamepad2.dpad_up)
//            {
//                if(clawAngle.CS != clawAngleController.clawAngleStatus.SPECIMEN) clawAngle.CS = clawAngleController.clawAngleStatus.SPECIMEN;
//                else clawAngle.CS = clawAngleController.clawAngleStatus.TRANSFER;
//            }
//
//            if (!previousGamepad2.dpad_left && currentGamepad2.dpad_left)
//                if (claw.CS != clawController.clawStatus.CLOSED) claw.CS = clawController.clawStatus.CLOSED;
//                    else claw.CS = clawController.clawStatus.OPENED;
//
//            if (!previousGamepad2.dpad_right && currentGamepad2.dpad_right) {
//                if (collectAngle.CS != collectAngleController.collectAngleStatus.TRANSFER) {
//                    collectAngle.CS = collectAngleController.collectAngleStatus.TRANSFER;
//                    fourbar.CS = fourbarController.fourbarStatus.TRANSFER;
//                    clawAngleController.CS = clawAngleController.clawAngleStatus.TRANSFER;
//                } else {
//                    collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;
//                    fourbar.CS = fourbarController.fourbarStatus.SPECIMEN;
//                    clawAngleController.CS = clawAngleController.clawAngleStatus.SPECIMEN;
//                }
//                    }

//                if (!previousGamepad2.dpad_down && currentGamepad2.dpad_down)
//                {
//                    collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;
//                    fourbar.CS = fourbarController.fourbarStatus.TRANSFER;
//                    clawAngle.CS = clawAngleController.clawAngleStatus.TRANSFER;
//                }
//
//                if (!previousGamepad2.dpad_up && currentGamepad2.dpad_up)
//                {
//                    claw.CS = clawController.clawStatus.CLOSED;
//                }
//
//                if (!previousGamepad2.dpad_left && currentGamepad2.dpad_left)
//                {
//                    collectAngle.CS = collectAngleController.collectAngleStatus.TRANSFER;
//                    fourbar.CS = fourbarController.fourbarStatus.SAMPLE;
//                    clawAngle.CS = clawAngleController.clawAngleStatus.SAMPlE_SCORE;
//                }
//
//                if (!previousGamepad2.dpad_right && currentGamepad2.dpad_right)
//                    fourbar.CS = fourbarController.fourbarStatus.SPECIMEN;
//
//                if(!previousGamepad2.triangle && currentGamepad2.triangle)
//                {
//                    if(collectAngle.CS != collectAngleController.collectAngleStatus.DRIVE) collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;
//                    else collectAngle.CS = collectAngleController.collectAngleStatus.COLLECT;
//                }

            /**
             * UPDATES
             */

//            clawAngle.update(r);
//            claw.update(r);
//            collectAngle.update(r);
//            //extendo.update(r,position_extendo,1,voltage);
//            fourbar.update(r);
           //lift.update(r,position_lift,voltage);
//            outtake.update(r,claw,lift,fourbar,clawAngle,transfer);
//            transfer.update(r,claw,collectAngle,outtake,extendo);


            double loop = System.nanoTime();

            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.addData("collectPwer", collectPower);
//            telemetry.addData("extendo pos",(r.extendo.getCurrentPosition()));
//            telemetry.addData("extendo", extendo.CS);
//            telemetry.addData("lift pos",r.collect.getCurrentPosition());
//            telemetry.addData("claw", claw.CS);
//           telemetry.addData("collect angle",collectAngle.CS);
//            telemetry.addData("fourbar",fourbar.CS);
//            telemetry.addData("claw angle", clawAngle.CS);
           // telemetry.addData("testfsm", org.firstinspires.ftc.teamcode.system_controllers.testfsm.CS);
          //  testfsm.update(fourbar, clawAngle);
//            telemetry.addData("lift" , r.collect.getCurrentPosition());
//            telemetry.addData("distanta senzor culoare", r.color_sensor.getDistance(DistanceUnit.MM));
//            telemetry.addData("stare senzor cuva", r.cuva_sensor.getState());

            telemetry.update();
        }
        }
    }

