package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Globals.globals;
import org.firstinspires.ftc.teamcode.Globals.robotMap;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;
import org.firstinspires.ftc.teamcode.system_controllers.clawAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.clawController;
import org.firstinspires.ftc.teamcode.system_controllers.collectAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.extendoController;
import org.firstinspires.ftc.teamcode.system_controllers.fourbarController;
import org.firstinspires.ftc.teamcode.system_controllers.liftController;
import org.firstinspires.ftc.teamcode.system_controllers.outtakeController;
import org.firstinspires.ftc.teamcode.system_controllers.transferController;

import java.util.Objects;

@Config
@Autonomous(group = "Auto", name = "LeftAutoRed")
public class LeftAuto extends LinearOpMode {
    enum STROBOT {
        START,
        PRElOAD,

        GO_TO_PRELOAD,

        GO_TO_PRELOAD_SCORE,
        PRELOAD_SCORE,

        GO_COLLECT_SAMPLE,

        FULL_EXTEND,

        CHECK_SAMPLE,
        CAN_GO_TO_SCORE_SAMPLE,
        GO_TO_SCORE_SAMPLE,
        SCORE_SAMPLE,

        GO_TO_COLLECT_SUMMERSIBLE,
        DETECT,
        COLLECT_SUMMERSIBLE,
        SWEEP_1,
        SWEEP_2,
        SWEEP_3,

        FAILSAFE_1,
        FAILSAFE_2,

        COLLECT_SUMMERSIBLE_CHECK,
        GO_TO_SCORE_SUMMERSIBLE,
        GO_SCORE_SUMMERSIBLE,
        SCORE_SUMMERSIBLE,
        GO_PARK,
        PARK,
        NOTHING
    }

    public Telemetry telemetryA;

    public static double preloadAngle = Math.toRadians(335);
    public static double yellow1Angle = Math.toRadians(343);
    public static double yellow2Angle = Math.toRadians(0);
    public static double yellow3Angle = Math.toRadians(28);
    public static double scoreAngle = Math.toRadians(338);
    public static double score1Angle = Math.toRadians(0);
    public static double score2Angle = Math.toRadians(0);
    public static double score3Angle = Math.toRadians(350);
    public static double parkAngle = Math.toRadians(90);
    public static double collectSummersibleAngle = Math.toRadians(270);
    boolean ok = false;
    boolean okAuto = false;



    @Override
    public void runOpMode() throws InterruptedException {
        Follower drive = new Follower(hardwareMap);
        robotMap r = new robotMap(hardwareMap);

        int liftpos;
        int extendopos;
        int poscnt = 0;

        /**
         * POZITII CUSTOM
         * RIGHT BUMPER ++
         * LEFT BUMPER --
         * X pentru ready
         */
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("init");
        telemetryA.update();
        while (!okAuto)
        {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            if (!previousGamepad1.right_bumper && currentGamepad1.right_bumper) poscnt++;
            if (!previousGamepad1.left_bumper && currentGamepad1.left_bumper) poscnt--;
            if (!previousGamepad1.triangle && currentGamepad1.triangle) okAuto= true;
            telemetryA.addData("current collect pos x",75+poscnt);
            telemetryA.addLine("daca dai start acum crapa");
            telemetryA.addLine("right bumper creste x colelct");
            telemetryA.addLine("left bumper scade x colelct");
            telemetryA.addLine("triughi when ready");
            telemetryA.update();
        }
        /**
         * POSES
         */

        Pose startPose = new Pose(8,117.5,Math.toRadians(0));

        drive.setStartingPose(startPose);

        Pose preloadPose = new Pose(22.5,136,preloadAngle);
        Pose preloadScorePose = new Pose(15.5, 138, preloadAngle);
        Pose yellowPose1 = new Pose(24.5,133,yellow1Angle);
        Pose yellowPose2 = new Pose(23,136,yellow2Angle);
        Pose yellowPose3 = new Pose(24.5,138,yellow3Angle);
        Pose scorePose1 = new Pose(16.5,141,score1Angle);
        Pose scorePose2 = new Pose(16.5,142,score2Angle);
        Pose scorePose3 = new Pose(17,141,score3Angle);
        Pose summersibleAlignPose1 = new Pose(60,120,collectSummersibleAngle);
        Pose summersibleCollectPose1 = new Pose(75+poscnt,100,collectSummersibleAngle);
        Pose summersibleScorePose1 = new Pose(20,141,0);
        Pose summersibleGoScorePose1 = new Pose(19,141,0);
        Pose summersibleAlignPose2 = new Pose(60,120,collectSummersibleAngle);
        Pose summersibleCollectPose2 = new Pose(75+poscnt,100,collectSummersibleAngle);
        Pose summersibleScorePose2 = new Pose(20,141,0);
        Pose summersibleGoScorePose2 = new Pose(19,141,0);
        Pose parkPose = new Pose(60,103,parkAngle);
        Pose parkScorePose = new Pose(55, 96, parkAngle);

        /**
         * PATHS
         */

        Path preload = new Path(new BezierLine(new Point(startPose), new Point(preloadPose)));
        preload.setLinearHeadingInterpolation(Math.toRadians(0), preloadAngle);

        Path preloadScore = new Path(new BezierLine(new Point(preloadPose), new Point(preloadScorePose)));
        preloadScore.setConstantHeadingInterpolation(preloadAngle);

        Path yellow1 = new Path(new BezierLine(new Point(preloadScorePose), new Point(yellowPose1)));
        yellow1.setLinearHeadingInterpolation(preloadAngle,yellow1Angle);

        Path score1 = new Path(new BezierLine(new Point(yellowPose1), new Point(scorePose1)));
        score1.setLinearHeadingInterpolation(yellow1Angle,score1Angle);

        Path yellow2 = new Path(new BezierLine(new Point(scorePose1), new Point(yellowPose2)));
        yellow2.setLinearHeadingInterpolation(score1Angle,yellow2Angle);

        Path score2 = new Path(new BezierLine(new Point(yellowPose2), new Point(scorePose2)));
        score2.setLinearHeadingInterpolation(yellow2Angle,score2Angle);

        Path yellow3 = new Path(new BezierLine(new Point(scorePose2), new Point(yellowPose3)));
        yellow3.setLinearHeadingInterpolation(score2Angle,yellow3Angle);

        Path score3 = new Path(new BezierLine(new Point(yellowPose3), new Point(scorePose3)));
        score3.setLinearHeadingInterpolation(yellow3Angle,score3Angle);

        Path summersibleAlign1 = new Path(new BezierCurve(new Point(scorePose3),new Point(summersibleAlignPose1),new Point(summersibleCollectPose1)));
        summersibleAlign1.setLinearHeadingInterpolation(score3Angle,collectSummersibleAngle);

        Path summersibleScore1 = new Path(new BezierCurve(new Point (summersibleCollectPose1),new Point(summersibleAlignPose1), new Point(summersibleScorePose1)));
        summersibleScore1.setLinearHeadingInterpolation(collectSummersibleAngle,score1Angle);

        Path summersibleGoScore1 = new Path(new BezierLine(new Point(summersibleScorePose1),new Point(summersibleGoScorePose1)));
        summersibleGoScore1.setLinearHeadingInterpolation(0,0);

        Path summersibleAlign2 = new Path(new BezierCurve(new Point(scorePose3),new Point(summersibleAlignPose2),new Point(summersibleCollectPose2)));
        summersibleAlign2.setLinearHeadingInterpolation(score3Angle,collectSummersibleAngle+Math.toRadians(10));

        Path summersibleScore2 = new Path(new BezierCurve(new Point (summersibleCollectPose2),new Point(summersibleAlignPose2), new Point(summersibleScorePose2)));
        summersibleScore2.setLinearHeadingInterpolation(collectSummersibleAngle,score1Angle);

        Path summersibleGoScore2 = new Path(new BezierLine(new Point(summersibleScorePose2),new Point(summersibleGoScorePose2)));
        summersibleGoScore2.setLinearHeadingInterpolation(0,0);

        Path park = new Path(new BezierLine(new Point(scorePose3), new Point(parkPose)));
        park.setLinearHeadingInterpolation(score3Angle,parkAngle);

        Path park_score = new Path(new BezierLine(new Point(parkPose), new Point(parkScorePose)));
        park_score.setConstantHeadingInterpolation(parkAngle);
        /**
         * INITS
         */

        clawController claw = new clawController();
        clawAngleController clawAngle = new clawAngleController();
        collectAngleController collectAngle = new collectAngleController();
        extendoController extendo = new extendoController();
        fourbarController fourbar = new fourbarController();
        liftController lift = new liftController();
        outtakeController outtake = new outtakeController();
        transferController transfer = new transferController();

        double voltage;
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage = batteryVoltageSensor.getVoltage();

        claw.CS = clawController.clawStatus.CLOSED;
        clawAngle.CS = clawAngleController.clawAngleStatus.TRANSFER;
        collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;
        extendo.CS = extendoController.extendoStatus.RETRACTED;
        fourbar.CS = fourbarController.fourbarStatus.TRANSFER;
        lift.CS = liftController.liftStatus.DOWN;
        //outtake.CS = outtakeController.outtakeStatus.INITIALIZE;

        double loopTime = 0;
        double limit_collect[] = {0.6, 0.6, 0.6};
        double limit_extend_full[] = {0.4, 0.4, 0.4};

        LeftAutoController leftAutoController = new LeftAutoController();

        leftAutoController.CurrentStatus = LeftAutoController.autoControllerStatus.NOTHING;

        /**
         * UPDATES
         */

        claw.update(r);
        clawAngle.update(r);
        collectAngle.update(r);
        extendo.update(r,0,0.5,voltage);
        fourbar.update(r);
        lift.update(r,0,voltage);
        //  outtake.update(r,claw,lift,fourbar,clawAngle,transfer);
        leftAutoController.update(r,clawAngle, claw, collectAngle, extendo, fourbar, lift);

        // drive.setStartingPose(startPose);


        STROBOT status = STROBOT.START;

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime timer_sample = new ElapsedTime();
        ElapsedTime timer_wait = new ElapsedTime();
        ElapsedTime timer_loop = new ElapsedTime();
        ElapsedTime timer_failsafe = new ElapsedTime();
        ElapsedTime timer_park = new ElapsedTime();
        ElapsedTime timer_outtake = new ElapsedTime();
        ElapsedTime timer_sweep  = new ElapsedTime();
        ElapsedTime timer_spit = new ElapsedTime();
        ElapsedTime fuuny_teren = new ElapsedTime();
        int nrcicluri = 0;
        int nrciclurisumersibil = 0;
        int nrturn = 0;

        telemetryA.addLine("init ready");
        telemetryA.update();
//        while (!isStarted() && !isStopRequested()) {
//
//
//            drive.update();
//            sleep(50);
//        }

        globals.extendo_auto_i =0;
        globals.is_transfer_summersible = false;
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {


            liftpos = r.collect.getCurrentPosition();
            extendopos = r.extendo.getCurrentPosition()*(-1);
//             FollowerConstants.holdPointHeadingScaling = 0.35;
//             FollowerConstants.holdPointTranslationalScaling = 0.25;
            switch (status)
            {
                case START:
                {
                    drive.setStartingPose(startPose);
                    status = STROBOT.GO_TO_PRELOAD;
                    timer.reset();
                    break;
                }

                case GO_TO_PRELOAD:
                {
                    drive.followPath(preload);
                    leftAutoController.CurrentStatus = LeftAutoController.autoControllerStatus.SAMPLE_START;
                    status = STROBOT.GO_TO_PRELOAD_SCORE;

                    break;
                }

                case GO_TO_PRELOAD_SCORE:
                {
                    if(!drive.isBusy())
                    {
                        drive.followPath(preloadScore);
                        timer_sample.reset();
                        ok = false;
                        status = STROBOT.PRELOAD_SCORE;

                    }
                    break;
                }

                case PRELOAD_SCORE:
                {
                    if(timer_sample.seconds()>0.3 && (LeftAutoController.CurrentStatus == LeftAutoController.autoControllerStatus.SAMPLE_START || LeftAutoController.CurrentStatus == LeftAutoController.autoControllerStatus.SAMPLE_FOURBAR || LeftAutoController.CurrentStatus == LeftAutoController.autoControllerStatus.SAMPLE_DONE))
                    {
                        leftAutoController.CurrentStatus = LeftAutoController.autoControllerStatus.COLLECT_SAMPLE;
                    }

                    if(timer_sample.seconds()>0.5)
                    {
                        leftAutoController.CurrentStatus2 = LeftAutoController.autoControllerStatus.OPEN_CLAW;
                        // timer_wait.reset();
                        status = STROBOT.GO_COLLECT_SAMPLE;
                    }
                    break;
                }

                case GO_COLLECT_SAMPLE:
                {
                    switch (nrcicluri)
                    {
                        case 0:
                        {
                            drive.followPath(yellow1);
                            timer_wait.reset();
                            ok = false;
                            status = STROBOT.FULL_EXTEND;
                            break;
                        }

                        case 1:
                        {
                            drive.followPath(yellow2);
                            timer_wait.reset();
                            ok = false;
                            status = STROBOT.FULL_EXTEND;
                            break;
                        }
                        case 2:
                        {
                            drive.followPath(yellow3);
                            timer_wait.reset();
                            ok = false;
                            status = STROBOT.FULL_EXTEND;
                            break;
                        }
                        case 3:
                        {
                            if (timer_wait.seconds() > 0.4) {
                                drive.followPath(summersibleAlign1);
                                timer_outtake.reset();
                                fuuny_teren.reset();
                                status = STROBOT.GO_TO_COLLECT_SUMMERSIBLE;
                            }
                            break;
                        }
                        case 4:
                        {
                            if (timer_wait.seconds() > 0.4) {
                                drive.followPath(summersibleAlign2);
                                timer_outtake.reset();
                                fuuny_teren.reset();
                                status = STROBOT.GO_TO_COLLECT_SUMMERSIBLE;
                            }
                            break;
                        }

                        case 5:
                        {
                            if (timer_wait.seconds() > 0.4) {
                                drive.followPath(summersibleAlign2);
                                timer_outtake.reset();
                                fuuny_teren.reset();
                                status = STROBOT.GO_TO_COLLECT_SUMMERSIBLE;
                            }
                            break;
                        }
                    }

                    break;
                }

                case FULL_EXTEND:
                {
                    if(timer_wait.seconds() > limit_extend_full[nrcicluri])
                    {
                        leftAutoController.CurrentStatus2 = LeftAutoController.autoControllerStatus.OUTTAKE_DOWN;

                    }
                    if(timer_wait.seconds() > limit_collect[nrcicluri])
                    {
                        leftAutoController.CurrentStatus = LeftAutoController.autoControllerStatus.COLLECT_SAMPLE_EXTEND_FULL;
                        timer_loop.reset();
                        timer_failsafe.reset();
                        status = STROBOT.CHECK_SAMPLE;
                    }

                    break;
                }

                case CHECK_SAMPLE:
                {
                    if(extendopos > 300) {
                        if(timer_loop.seconds()>0.05)
                        {
                            timer_loop.reset();
                            if (Objects.equals(Retrun_Color(r.color_sensor), "yellow")) {
                                leftAutoController.CurrentStatus = LeftAutoController.autoControllerStatus.TRANSFER_BEGIN;
                                status = STROBOT.CAN_GO_TO_SCORE_SAMPLE;
                            }
                        }
                    }
                    if(timer_failsafe.seconds()>0.5)
                    {
                        leftAutoController.CurrentStatus = LeftAutoController.autoControllerStatus.TRANSFER_BEGIN;
                        status = STROBOT.CAN_GO_TO_SCORE_SAMPLE;

                    }
                    break;
                }

                case CAN_GO_TO_SCORE_SAMPLE:
                {
                    if(leftAutoController.CurrentStatus == LeftAutoController.autoControllerStatus.TRANSFER_DONE)
                    {
                        leftAutoController.CurrentStatus = LeftAutoController.autoControllerStatus.SAMPLE_START;
                        timer_sample.reset();
                        status = STROBOT.GO_TO_SCORE_SAMPLE;
                    }
                    break;
                }

                case GO_TO_SCORE_SAMPLE:
                {
                    if(timer_sample.seconds() > 0.85)
                    {switch (nrcicluri)
                    {
                        case 0:
                        {
                            drive.followPath(score1);
                            fuuny_teren.reset();
                            status = STROBOT.SCORE_SAMPLE;
                            break;
                        }
                        case 1:
                        {
                            drive.followPath(score2);
                            fuuny_teren.reset();
                            status = STROBOT.SCORE_SAMPLE;
                            break;
                        }
                        case 2:
                        {
                            drive.followPath(score3);
                            fuuny_teren.reset();
                            status = STROBOT.SCORE_SAMPLE;
                            break;
                        }
                    }
                    }
                    break;
                }

                case SCORE_SAMPLE:
                {
                    if(leftAutoController.CurrentStatus == LeftAutoController.autoControllerStatus.SAMPLE_DONE && (!drive.isBusy() || fuuny_teren.seconds() > 1))
                    {
                        leftAutoController.CurrentStatus2 = LeftAutoController.autoControllerStatus.OPEN_CLAW;
                        nrcicluri++;
                        globals.extendo_auto_i++;
                        //timer_wait.reset();
                        if (nrcicluri<3) leftAutoController.CurrentStatus = LeftAutoController.autoControllerStatus.COLLECT_SAMPLE;
                        else timer_wait.reset();
                        status = STROBOT.GO_COLLECT_SAMPLE;
                    }
                    break;
                }
                case GO_TO_COLLECT_SUMMERSIBLE:
                {
                    //drive.followPath(summersibleAlign1);
                    if (timer_outtake.seconds() > 0.4)
                    {
                        leftAutoController.CurrentStatus2 = LeftAutoController.autoControllerStatus.OUTTAKE_DOWN;
                    }
                    if (!drive.isBusy() || fuuny_teren.seconds() > 1.5)
                    {
                        leftAutoController.CurrentStatus = LeftAutoController.autoControllerStatus.COLLECT_SUMMERSIBLE_SHORT;
                        status = STROBOT.COLLECT_SUMMERSIBLE;//status = STROBOT.SWEEP_1;
                    }
                    break;
                }
                case DETECT:
                {
                    //todo introdu secventa limelight
                    status = STROBOT.COLLECT_SUMMERSIBLE;
                    break;
                }
                case SWEEP_1:
                {
                    if (extendopos > 50)
                    {
                        collectAngleController.CS = collectAngleController.collectAngleStatus.SWEEP;
                        drive.holdPoint(new BezierPoint(new Point(summersibleCollectPose1.getX(),summersibleCollectPose1.getY(),Point.CARTESIAN)),Math.toRadians(350));
                        timer_sweep.reset();
                        status = STROBOT.SWEEP_2;
                    }
                    break;
                }
                case SWEEP_2:
                {
                    if (!drive.isBusy() && timer_sweep.seconds() > 0.5)
                    {
                        drive.holdPoint(new BezierPoint(new Point(summersibleCollectPose1.getX(),summersibleCollectPose1.getY(),Point.CARTESIAN)),Math.toRadians(260));
                        timer_sweep.reset();
                        status = STROBOT.SWEEP_3;
                    }
                    break;
                }
                case SWEEP_3:
                {
                    leftAutoController.CurrentStatus = LeftAutoController.autoControllerStatus.SUMMERSIBLE_RETRACT;
                    if (!drive.isBusy() && timer_sweep.seconds() > 0.5)
                    {
                        status = STROBOT.COLLECT_SUMMERSIBLE;
                    }
                    break;
                }
                case COLLECT_SUMMERSIBLE:
                {
                    if (!drive.isBusy() || fuuny_teren.seconds() > 1.5)
                    {
                        leftAutoController.CurrentStatus = LeftAutoController.autoControllerStatus.COLLECT_SUMMERSIBLE_EXTEND_SHORT;
                        timer_failsafe.reset();
                        status = STROBOT.COLLECT_SUMMERSIBLE_CHECK;
                    }
                    break;
                }
                case COLLECT_SUMMERSIBLE_CHECK:
                {
                    if (Objects.equals(Retrun_Color(r.color_sensor), "yellow") || Objects.equals(Retrun_Color(r.color_sensor), "red"))
                    {
                        leftAutoController.CurrentStatus = LeftAutoController.autoControllerStatus.TRANSFER_BEGIN;
                        globals.is_transfer_summersible = true;
                        status = STROBOT.GO_TO_SCORE_SUMMERSIBLE;
                    }
                    if (timer_failsafe.seconds() > 0.7 || Objects.equals(Retrun_Color(r.color_sensor), "blue")) status = STROBOT.FAILSAFE_1;
                    break;
                }

                case FAILSAFE_1:
                {
                    timer_spit.reset();
                    //LeftAutoController.CurrentStatus = LeftAutoController.autoControllerStatus.RETRACT_SHORT;
                    status = STROBOT.FAILSAFE_2;
                    break;
                }

                case FAILSAFE_2:
                {
                    if (Objects.equals(Retrun_Color(r.color_sensor), "blue")) r.collect.setPower(0.7);
                    else r.collect.setPower(-0.9);
                    if(timer_spit.seconds()>0.5)
                    {
                        nrturn++;
                        drive.holdPoint(new BezierPoint(new Point(summersibleCollectPose1.getX(),summersibleCollectPose1.getY(),Point.CARTESIAN)),summersibleCollectPose1.getHeading()+Math.toRadians(15*(nrturn+1)/2*((nrturn%2==0)?-1:1)));
                        status = STROBOT.COLLECT_SUMMERSIBLE;
                    }
                    break;
                }

                case GO_TO_SCORE_SUMMERSIBLE:
                {
                    drive.followPath(summersibleScore1);
                    status = STROBOT.GO_SCORE_SUMMERSIBLE;
                    break;
                }
                // 2 caseuri ca n-am timp de lift up 💅🏼💅🏼
                case GO_SCORE_SUMMERSIBLE:
                {
                    if (liftpos > 24000)
                    {
                        drive.followPath(summersibleGoScore1);
                        timer_wait.reset();
                        status = STROBOT.SCORE_SUMMERSIBLE;
                    }
                    break;
                }
                case SCORE_SUMMERSIBLE:
                {
                    if (!drive.isBusy() && timer_wait.seconds() > 0.7)
                    {
                        clawController.CS = clawController.clawStatus.OPENED;
                        timer_park.reset();
                        drive.followPath(park);
                        if (nrcicluri == 3 && timer.seconds() < 25 )
                        {
                            nrcicluri++;
                            status = STROBOT.GO_COLLECT_SAMPLE;
                        }
                        else status = STROBOT.GO_PARK;
                    }
                    break;
                }
                case GO_PARK:
                {
                    if (timer_park.seconds() > 0.7)
                    {
                        leftAutoController.CurrentStatus2 = LeftAutoController.autoControllerStatus.HANG_LV1;
                        status = STROBOT.PARK;
                    }
                    break;
                }
                case PARK:
                {
                    if (!drive.isBusy())
                    {
                        drive.followPath(park_score);
                        status = STROBOT.NOTHING;
                    }
                    break;
                }

            }

            claw.update(r);
            clawAngle.update(r);
            collectAngle.update(r);
            extendo.update(r,extendopos,0.7,voltage);
            fourbar.update(r);
            lift.update(r,liftpos,voltage);
            // outtake.update(r,claw,lift,fourbar,clawAngle,transfer);
            transfer.update(r,claw,collectAngle,outtake,extendo);

            leftAutoController.update(r,clawAngle, claw, collectAngle, extendo, fourbar, lift);

            double loop = System.nanoTime();

//            telemetryA.addData("caz", status);
//            telemetryA.addData("time",timer);
//            try
//            {telemetryA.addData("x", drive.getPose().getX());
//            telemetryA.addData("y", drive.getPose().getY());} catch (Exception e){}
//
//
//            telemetryA.update();
            telemetry.addData("hz", 1000000000 / (loop - loopTime));
            telemetry.addData("status", status);
            telemetry.addData("systems", LeftAutoController.CurrentStatus);
            telemetry.addData("systems2", LeftAutoController.CurrentStatus2);
            telemetry.addData("extendopos",extendopos);
            ;
            //
            //telemetry.addData("globals i ", org.firstinspires.ftc.teamcode.Globals.globals.extendo_auto_i);
            //telemetry.addData("timer loop",timer_loop.seconds());

            telemetry.addData("liftpos", liftpos);
            telemetry.addData("poscnt",poscnt);
            telemetry.update();
            loopTime = loop;
            drive.update();
            //drive.telemetryDebug(telemetryA);

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
        else if(green_value > blue_value && green_value > 300)
        {
            return "yellow";
        }
        else if(blue_value > 300)
        {

            return "blue";
        }
        else {return  "nothing";}
    }
}