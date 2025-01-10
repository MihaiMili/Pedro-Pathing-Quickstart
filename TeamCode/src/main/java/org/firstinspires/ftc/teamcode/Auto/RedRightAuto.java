package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Globals.globals;
import org.firstinspires.ftc.teamcode.Globals.robotMap;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.DashboardPoseTracker;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;
import org.firstinspires.ftc.teamcode.system_controllers.clawAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.clawController;
import org.firstinspires.ftc.teamcode.system_controllers.collectAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.extendoController;
import org.firstinspires.ftc.teamcode.system_controllers.fourbarController;
import org.firstinspires.ftc.teamcode.system_controllers.liftController;
import org.firstinspires.ftc.teamcode.system_controllers.outtakeController;
import org.firstinspires.ftc.teamcode.system_controllers.transferController;

@Config
@Autonomous(group = "Auto", name = "RedRightAuto")
public class RedRightAuto extends LinearOpMode {
    enum STROBOT {
        START,
        GO_TO_PRELOAD_SCORE,
        GO_TO_COLOR_COLLECT,
        COLLECT,
        COLLECT_CHECK,
        GO_TO_COLOR_SPIT,
        COLOR_SPIT,
        COLOR_SPIT_2,
        GO_COLLECT_SPECIMEN,

        COLLECT_SPECIMEN,

        GO_SCORE_SPECIMEN,
        SCORE_SPECIMEN,
        SCORE_CYCLE,

        PARK,
        NOTHING,

        SCORE_STATE_1,
        SCORE_STATE_2,

        FUUNY_HP_1,
        FUNNY_HP_2,

        FUNNY_HP_3,


    }

    public Telemetry telemetryA;
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;
    public static double scoreAngle = Math.toRadians(0);
    public static double color1CollectAngle = Math.toRadians(323);
    public static double color2CollectAngle = Math.toRadians(315);
    public static double color3CollectAngle = Math.toRadians(315);
    public static double color1SpitAngle = Math.toRadians(250);
    public static double color2SpitAngle = Math.toRadians(240);
    public static double color3SpitAngle = Math.toRadians(240);
    public static double specimenCollectAngle = Math.toRadians(0);
    public static double parkAngle = Math.toRadians(0);


    public static double limit_collect[] = {1,0.55,0.55};
    public static double limit_spit[] = {1,1,1};
    public PathChain specimenGrab1,specimenGrab2,specimenGrab3,specimenGrab4;



    public
    boolean ok = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Follower drive = new Follower(hardwareMap);
        robotMap r = new robotMap(hardwareMap);

        Follower.drawOnDashboard = true;

        poseUpdater = new PoseUpdater(hardwareMap);

        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        int liftpos;
        int extendopos;
        /**
         * POSES
         */

        Pose startPose = new Pose(8,64,Math.toRadians(0));


        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        drive.setStartingPose(startPose);

        poseUpdater.setStartingPose(startPose);
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();

//        Pose preloadScorePose = new Pose(41,74,scoreAngle);
//        Pose color1CollectPose = new Pose (33,37,color1CollectAngle);
//        Pose color2CollectPose = new Pose (33,31,color2CollectAngle);
//        Pose color3CollectPose = new Pose(25,25,color3CollectAngle);
//        Pose color1SpitPose = new Pose(29,29,color1SpitAngle);
//        Pose color2SpitPose = new Pose(27,27,color2SpitAngle);
//        Pose color3SpitPose = new Pose(23,23,color3SpitAngle);
//        Pose specimenCollectPose = new Pose(11,35,specimenCollectAngle);
//        Pose specimen1ScorePose = new Pose(41,72,scoreAngle);
//        Pose specimen2ScorePose = new Pose(42,70,scoreAngle);
//        Pose specimen3ScorePose = new Pose(42,68,scoreAngle);
//        Pose specimen4ScorePose = new Pose(42,66,scoreAngle);
//        Pose parkPose = new Pose(11,30,parkAngle);

        Pose preloadScorePose = new Pose(40.5,70,scoreAngle);
        Pose color1CollectPose = new Pose (30,33,color1CollectAngle);
        Pose color2CollectPose = new Pose (29,28,color2CollectAngle);
        Pose color3CollectPose = new Pose(30,17,color3CollectAngle);
        Pose color1SpitPose = new Pose(31,37,color1SpitAngle);
        Pose color2SpitPose = new Pose(32,26,color2SpitAngle);
        Pose color3SpitPose = new Pose(32,16,color3SpitAngle);
        Pose specimen1AlignPose = new Pose(25,32,specimenCollectAngle);
        Pose specimen1CollectPose = new Pose(11,35,specimenCollectAngle);
        Pose specimen2AlignPose = new Pose(26.5,42,specimenCollectAngle);//27 42
        Pose specimen2CollectPose = new Pose(11,35,specimenCollectAngle);
        Pose specimen3AlignPose = new Pose(26.5,42,specimenCollectAngle);//27 42
        Pose specimen3CollectPose = new Pose(11,35,specimenCollectAngle);
        Pose specimen4AlignPose = new Pose(26.5,42,specimenCollectAngle);
        Pose specimen4CollectPose = new Pose(11,35,specimenCollectAngle);
        Pose specimen1ScorePose = new Pose(41.9,70,scoreAngle);
        Pose specimen2ScorePose = new Pose(41.9,69,scoreAngle);
        Pose specimen3ScorePose = new Pose(41.9,68,scoreAngle);
        Pose specimen4ScorePose = new Pose(41.9,67,scoreAngle);

        Pose parkPose = new Pose(11,33,parkAngle);



        /**
         * PATHS
         */

        Path preload = new Path(new BezierLine(new Point(startPose), new Point(preloadScorePose)));
        preload.setConstantHeadingInterpolation(Math.toRadians(0));

        Path color1collect = new Path((new BezierLine(new Point(preloadScorePose),new Point(color1CollectPose))));
        color1collect.setLinearHeadingInterpolation(scoreAngle,color1CollectAngle);

        Path color1spit = new Path((new BezierLine(new Point(color1CollectPose),new Point(color1SpitPose))));
        color1spit.setLinearHeadingInterpolation(color1CollectPose.getHeading(),color1SpitPose.getHeading());

        Path color2collect = new Path((new BezierLine(new Point(color1SpitPose),new Point(color2CollectPose))));
        color2collect.setLinearHeadingInterpolation(color1SpitPose.getHeading(),color2CollectPose.getHeading(),0);

        Path color2spit = new Path((new BezierLine(new Point(color2CollectPose),new Point(color2SpitPose))));
        color2spit.setLinearHeadingInterpolation(color2CollectPose.getHeading(),color2SpitPose.getHeading(),0);

        Path color3collect = new Path((new BezierLine(new Point(color2SpitPose),new Point(color3CollectPose))));
        color3collect.setLinearHeadingInterpolation(color2SpitAngle,color3CollectAngle, 0);

        Path color3spit = new Path((new BezierLine(new Point(color3CollectPose),new Point(color3SpitPose))));
        color3spit.setLinearHeadingInterpolation(color3CollectAngle,color3SpitAngle,0);

        Path specimen1Align = new Path(new BezierLine(new Point(color3SpitPose),new Point(specimen1AlignPose)));
        specimen1Align.setLinearHeadingInterpolation(color3SpitAngle,specimenCollectAngle,0);

        Path specimen1Collect = new Path(new BezierLine(new Point(specimen1AlignPose),new Point(specimen1CollectPose)));
        specimen1Collect.setConstantHeadingInterpolation(specimenCollectAngle);

        Path specimen1Score = new Path(new BezierLine(new Point(specimen1CollectPose),new Point(specimen1ScorePose)));
        specimen1Score.setConstantHeadingInterpolation(specimenCollectAngle);

        Path specimen2Align = new Path(new BezierLine(new Point(specimen1ScorePose),new Point(specimen2AlignPose)));
        specimen2Align.setConstantHeadingInterpolation(specimenCollectAngle);

        Path specimen2Collect = new Path(new BezierLine(new Point(specimen2AlignPose),new Point(specimen2CollectPose)));
        specimen2Collect.setConstantHeadingInterpolation(specimenCollectAngle);

        Path specimen2Score = new Path(new BezierLine(new Point(specimen2CollectPose),new Point(specimen2ScorePose)));
        specimen2Score.setConstantHeadingInterpolation(specimenCollectAngle);

        Path specimen3Align = new Path(new BezierLine(new Point(specimen2ScorePose),new Point(specimen3AlignPose)));
        specimen3Align.setConstantHeadingInterpolation(specimenCollectAngle);

        Path specimen3Collect = new Path(new BezierLine(new Point(specimen3AlignPose),new Point(specimen3CollectPose)));
        specimen3Collect.setConstantHeadingInterpolation(specimenCollectAngle);

        Path specimen3Score = new Path(new BezierLine(new Point(specimen3CollectPose),new Point(specimen3ScorePose)));
        specimen3Score.setConstantHeadingInterpolation(specimenCollectAngle);

        Path specimen4Align = new Path(new BezierLine(new Point(specimen3ScorePose),new Point(specimen4AlignPose)));
        specimen4Align.setConstantHeadingInterpolation(specimenCollectAngle);

        Path specimen4Collect = new Path(new BezierLine(new Point(specimen4AlignPose),new Point(specimen4CollectPose)));
        specimen4Collect.setConstantHeadingInterpolation(specimenCollectAngle);

        Path specimen4Score = new Path(new BezierLine(new Point(specimen4CollectPose),new Point(specimen4ScorePose)));
        specimen4Score.setConstantHeadingInterpolation(specimenCollectAngle);

        Path park = new Path(new BezierLine(new Point(specimen4ScorePose),new Point(parkPose)));
        park.setLinearHeadingInterpolation(scoreAngle,parkAngle) ;

        Path specimen2curva = new Path(new BezierCurve(new Point(specimen1ScorePose),new Point(specimen2AlignPose),new Point(specimen2CollectPose)));
        specimen2curva.setLinearHeadingInterpolation(0,0);

        Path specimen3curva = new Path(new BezierCurve(new Point(specimen2ScorePose),new Point(specimen3AlignPose),new Point(specimen3CollectPose)));
        specimen3curva.setLinearHeadingInterpolation(0,0);

        Path specimen4curva = new Path(new BezierCurve(new Point(specimen3ScorePose),new Point(specimen4AlignPose),new Point(specimen4CollectPose)));
        specimen4curva.setLinearHeadingInterpolation(0,0);



        specimenGrab1 = drive.pathBuilder()
                .addPath(specimen1Align)
                .addPath(specimen1Collect)
                .build();

        specimenGrab2 = drive.pathBuilder()
                .addPath(specimen2Align)
                .addPath(specimen2Collect)
                //.addPath(specimen2curva)
                .build();

        specimenGrab3 = drive.pathBuilder()
                .addPath(specimen3Align)
                .addPath(specimen3Collect)
//                .addPath(specimen3curva)
                .build();
        specimenGrab4 = drive.pathBuilder()
                .addPath(specimen4Align)
                .addPath(specimen4Collect)
//                .addPath(specimen4curva)
                .build();




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
        clawAngle.CS = clawAngleController.clawAngleStatus.SPECIMEN;
        collectAngle.CS = collectAngleController.collectAngleStatus.DRIVE;
        extendo.CS = extendoController.extendoStatus.RETRACTED;
        fourbar.CS = fourbarController.fourbarStatus.SPECIMEN;
        lift.CS = liftController.liftStatus.DOWN;

        //outtake.CS = outtakeController.outtakeStatus.INITIALIZE;

        double loopTime = 0;
//        double limit_collect[] = {0.7, 0.7, 1.1};
//        double limit_extend_full[] = {0.6, 0.6, 1};
        RightAutoController rightAutoController = new RightAutoController();

        rightAutoController.CurrentStatus = RightAutoController.autoControllerStatus.NOTHING;

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
        rightAutoController.update(r,clawAngle, claw, collectAngle, extendo, fourbar, lift);

        // drive.setStartingPose(startPose);


        STROBOT status = STROBOT.START;

        /**
         * TIMERS
         */

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime timer_loop = new ElapsedTime();
        ElapsedTime score_timer = new ElapsedTime();
        ElapsedTime extendo_timer = new ElapsedTime();
        ElapsedTime timer_failsafe = new ElapsedTime();
        ElapsedTime timer_spit = new ElapsedTime();
        ElapsedTime timer_claw = new ElapsedTime();
        ElapsedTime timer_help_pls = new ElapsedTime();
        ElapsedTime timer_spit_2 = new ElapsedTime();
        ElapsedTime funny_hp_3 = new ElapsedTime();
        ElapsedTime funny_teren = new ElapsedTime();
        ElapsedTime timer_park = new ElapsedTime();
        int nrcicluri = 0,nrcicluriscorare = 0;

        /**
         * TELEMETRY
         */

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("init");
        telemetryA.update();

//        while (!isStarted() && !isStopRequested()) {
//
//
//            drive.update();
//            sleep(50);
//        }

        globals.extendo_auto_i = 0;
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {


            liftpos = r.collect.getCurrentPosition();
            extendopos = r.extendo.getCurrentPosition()*(-1);

            switch (status)
            {
                case START:
                {
                    timer_park.reset();

                    drive.setStartingPose(startPose);
                    status = STROBOT.GO_TO_PRELOAD_SCORE;
                    break;
                }

                case GO_TO_PRELOAD_SCORE:
                {
                    drive.followPath(preload);
                    funny_teren.reset();
                    rightAutoController.CurrentStatus = RightAutoController.autoControllerStatus.SPECIMEN_START;
                    score_timer.reset();
                    status = STROBOT.SCORE_STATE_1;
                    break;
                }

                case SCORE_STATE_1:
                {
                    if (!drive.isBusy() || funny_teren.seconds() > 3)
                    {
                        rightAutoController.CurrentStatus = RightAutoController.autoControllerStatus.SPECIMEN_SCORE;
                        score_timer.reset();
                        status = STROBOT.SCORE_STATE_2;
                    }
                    break;
                }

                case SCORE_STATE_2:
                {
                    if (score_timer.seconds() > 0.4)
                    {
                        status = STROBOT.GO_TO_COLOR_COLLECT;
                    }
                    break;
                }

                case GO_TO_COLOR_COLLECT:
                {
                    switch (nrcicluri)
                    {
                        case 0:
                        {
                            rightAutoController.CurrentStatus = RightAutoController.autoControllerStatus.OUTTAKE_DOWN;
                            drive.followPath(color1collect);

                            extendo_timer.reset();
                            status = STROBOT.COLLECT;
                            break;
                        }
                        case 1:
                        {
                            drive.followPath(color2collect);
                            extendo_timer.reset();
                            status = STROBOT.COLLECT;
                            break;
                        }
                        case 2:
                        {
                            drive.followPath(color3collect);
                            extendo_timer.reset();
                            status = STROBOT.COLLECT;
                            break;
                        }
                        case 3:
                        {
                            status = STROBOT.GO_COLLECT_SPECIMEN;
                            break;
                        }
                    }
                    break;
                }

                case COLLECT:
                {
                    timer_failsafe.reset();

                    if (extendo_timer.seconds() > 0.8 && nrcicluri == 0)
                    {
                        RightAutoController.CurrentStatus2 = RightAutoController.autoControllerStatus.SHORT_EXTEND;
                    }

                    if (!drive.isBusy() && extendo_timer.seconds() > limit_collect[nrcicluri]) {
                        r.collect.setPower(-1);
                        RightAutoController.CurrentStatus2 = RightAutoController.autoControllerStatus.FULL_EXTEND;
                        timer_failsafe.reset();
                        status = STROBOT.COLLECT_CHECK;
                    }

                    break;
                }

                case COLLECT_CHECK:
                {
                    if (extendopos > 350)
                    { if (Retrun_Color(r.color_sensor) == "red")
                    {
                        rightAutoController.CurrentStatus2 = RightAutoController.autoControllerStatus.SHORT_EXTEND;
                        status = STROBOT.GO_TO_COLOR_SPIT;

                    }
                    }
                    if(timer_failsafe.seconds() > 1) {
                        rightAutoController.CurrentStatus2 = RightAutoController.autoControllerStatus.SHORT_EXTEND;

                        status = STROBOT.GO_TO_COLOR_SPIT;

                    }
                    break;
                }

                case GO_TO_COLOR_SPIT:
                {
                    switch (nrcicluri)
                    {
                        case 0:
                        {
                            drive.holdPoint(new BezierPoint(new Point(color1CollectPose.getX(),color1CollectPose.getY(),Point.CARTESIAN)),Math.toRadians(220));
                            timer_spit.reset();
                            status = STROBOT.COLOR_SPIT;
                            break;
                        }

                        case 1:
                        {
                            drive.holdPoint(new BezierPoint(new Point(color2CollectPose.getX(),color2CollectPose.getY(),Point.CARTESIAN)),Math.toRadians(220));
                            timer_spit.reset();
                            status = STROBOT.COLOR_SPIT;

                            break;
                        }

                        case 2:
                        {
                            drive.holdPoint(new BezierPoint(new Point(color3CollectPose.getX(),color3CollectPose.getY(),Point.CARTESIAN)),Math.toRadians(200));
                            timer_spit.reset();
                            status = STROBOT.COLOR_SPIT;
                            break;
                        }

                        case 3:
                        {
                            timer_spit.reset();
                            status = STROBOT.COLOR_SPIT;
                            break;
                        }

                    }
                    break;
                }

                case COLOR_SPIT:
                {
                    if(timer_spit.seconds() > limit_spit[nrcicluri] - 0.3)
                    {
                        RightAutoController.CurrentStatus2 = RightAutoController.autoControllerStatus.FULL_EXTEND;

                    }
                    if (!drive.isBusy() && timer_spit.seconds() > limit_spit[nrcicluri]) {

                        RightAutoController.CurrentStatus = RightAutoController.autoControllerStatus.COLOR_SPIT;
                        // r.collect.setPower(0.85);
                        timer_spit_2.reset();
                        status = STROBOT.COLOR_SPIT_2;
                    }

                    break;
                }

                case COLOR_SPIT_2:
                {
                    if(nrcicluri<2)
                    {
                        if(timer_spit_2.seconds() > 0.45)
                        {
                            nrcicluri++;
                            globals.extendo_auto_i++;
                            extendo_timer.reset();
                            // r.collect.setPower(0);
                            RightAutoController.CurrentStatus2 = RightAutoController.autoControllerStatus.SHORT_EXTEND;
                            status = STROBOT.GO_TO_COLOR_COLLECT;
                        }
                    } else
                    {
                        if(timer_spit_2.seconds() > 0.45)
                        {
                            extendo_timer.reset();
                            status = STROBOT.GO_COLLECT_SPECIMEN;
                        }
                    }
                    break;
                }

                //TODO add cases
                case GO_COLLECT_SPECIMEN:
                {
                    if (timer_park.seconds() > 29)
                    {
                        drive.followPath(park);
                        status = STROBOT.PARK;
                    }
                    switch (nrcicluriscorare)
                    {
                        case 0:
                        {
                            //if (extendo_timer.seconds() > 1) r.collect.setPower(0);
                            drive.followPath(specimenGrab1,true);
                            RightAutoController.CurrentStatus2 = RightAutoController.autoControllerStatus.RETRACT;
                            fourbarController.CS = fourbarController.fourbarStatus.COLLECT_SPECIMEN;
                            status = STROBOT.COLLECT_SPECIMEN;
                            break;
                        }

                        case 1:
                        {
                            drive.followPath(specimenGrab2, true);
                            fourbarController.CS = fourbarController.fourbarStatus.COLLECT_SPECIMEN;
                            status = STROBOT.COLLECT_SPECIMEN;
                            break;
                        }

                        case 2:
                        {
                            drive.followPath(specimenGrab3, true);
                            fourbarController.CS = fourbarController.fourbarStatus.COLLECT_SPECIMEN;
                            status = STROBOT.COLLECT_SPECIMEN;
                            break;
                        }

                        case 3:
                        {
                            drive.followPath(specimenGrab4,true);
                            fourbarController.CS = fourbarController.fourbarStatus.COLLECT_SPECIMEN;
                            status = STROBOT.COLLECT_SPECIMEN;
                            break;
                        }

                        case 4:
                        {
                            drive.followPath(park);
                            status = STROBOT.PARK;
                            break;
                        }
                    }
                    break;
                }
                // TODO : delay ??
                case COLLECT_SPECIMEN:
                {
                    if (timer_park.seconds() > 29)
                    {
                        drive.followPath(park);
                        status = STROBOT.PARK;
                    }
                    //if (nrcicluriscorare > 0) rightAutoController.CurrentStatus = RightAutoController.autoControllerStatus.OUTTAKE_DOWN;
                    if (!drive.isBusy())
                    {
                        clawController.CS = clawController.clawStatus.CLOSED;
                        timer_claw.reset();
                        status = STROBOT.GO_SCORE_SPECIMEN;
                    }
                    break;
                }

                case FUUNY_HP_1:
                {
                    clawController.CS = clawController.clawStatus.OPENED;
                    Path FUNNY1 = new Path(new BezierLine(new Point(drive.getPose().getX(), drive.getPose().getY(), Point.CARTESIAN), new Point(drive.getPose().getX() +5, drive.getPose().getY(), Point.CARTESIAN)));
                    FUNNY1.setConstantHeadingInterpolation(Math.toRadians(0));
                    drive.followPath(FUNNY1);
                    status= STROBOT.FUNNY_HP_2;
                    break;
                }

                case FUNNY_HP_2:
                {
                    if(!drive.isBusy())
                    {
                        funny_hp_3.reset();
                        status = STROBOT.FUNNY_HP_3;
                    }
                    break;
                }

                case FUNNY_HP_3:
                {
                    if(funny_hp_3.seconds() > 0.2)
                    {
                        Path FUNNY2 = new Path(new BezierLine(new Point(drive.getPose().getX(), drive.getPose().getY(), Point.CARTESIAN), new Point(drive.getPose().getX() -5, drive.getPose().getY(), Point.CARTESIAN)));
                        FUNNY2.setConstantHeadingInterpolation(Math.toRadians(0));
                        drive.followPath(FUNNY2);
                        status= STROBOT.COLLECT_SPECIMEN;
                    }
                    break;
                }

                case GO_SCORE_SPECIMEN:
                {
                    if (timer_park.seconds() > 29)
                    {
                        drive.followPath(park);
                        status = STROBOT.PARK;
                    }
                    //if (timer_claw.seconds() > 0.5 && !HasSpecimen(r.claw_sensor)) status = RedRightAuto.STROBOT.FUUNY_HP_1;
                    if(timer_claw.seconds() > 0.2) {
                        switch (nrcicluriscorare) {
                            case 0: {
                                nrcicluriscorare++;
                                drive.followPath(specimen1Score);
                                funny_teren.reset();
                                RightAutoController.CurrentStatus = RightAutoController.autoControllerStatus.SPECIMEN_START;
                                //fourbarController.CS = org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.SPECIMEN;
                                status = STROBOT.SCORE_SPECIMEN;
                                break;
                            }
                            case 1: {
                                nrcicluriscorare++;
                                drive.followPath(specimen2Score);
                                funny_teren.reset();
                                RightAutoController.CurrentStatus = RightAutoController.autoControllerStatus.SPECIMEN_START;
                                //fourbarController.CS = org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.SPECIMEN;
                                status = STROBOT.SCORE_SPECIMEN;
                                break;
                            }
                            case 2: {
                                nrcicluriscorare++;
                                drive.followPath(specimen3Score);
                                funny_teren.reset();
                                RightAutoController.CurrentStatus = RightAutoController.autoControllerStatus.SPECIMEN_START;
                                //fourbarController.CS = org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.SPECIMEN;
                                status = STROBOT.SCORE_SPECIMEN;
                                break;
                            }
                            case 3:{
                                nrcicluriscorare++;
                                drive.followPath(specimen4Score);
                                funny_teren.reset();
                                RightAutoController.CurrentStatus = RightAutoController.autoControllerStatus.SPECIMEN_START;
                                //fourbarController.CS = org.firstinspires.ftc.teamcode.system_controllers.fourbarController.fourbarStatus.SPECIMEN;
                                status = STROBOT.SCORE_SPECIMEN;
                                break;
                            }
                        }
                    }
                    break;
                }
                case SCORE_SPECIMEN:
                {
                    if (timer_park.seconds() > 29)
                    {
                        drive.followPath(park);
                        status = STROBOT.PARK;
                    }
                    if (!drive.isBusy() || funny_teren.seconds()>2)
                    {
                        score_timer.reset();
                        rightAutoController.CurrentStatus = RightAutoController.autoControllerStatus.SPECIMEN_SCORE_CYCLE;
                        status = STROBOT.SCORE_CYCLE;
                    }
                    break;
                }
                case SCORE_CYCLE:
                {
                    if (timer_park.seconds() > 29)
                    {
                        drive.followPath(park);
                        status = STROBOT.PARK;
                    }
                    if (score_timer.seconds() > 1.3 || rightAutoController.CurrentStatus == RightAutoController.autoControllerStatus.SPECIMEN_SCORE_DONE)
                    {
                        //clawController.CS = clawController.clawStatus.OPENED;
                        rightAutoController.CurrentStatus = RightAutoController.autoControllerStatus.OUTTAKE_DOWN;
                        status = STROBOT.GO_COLLECT_SPECIMEN;
                    }
                    break;
                }
                case PARK:
                {
                    clawController.CS = clawController.clawStatus.OPENED;
                    fourbarController.CS = fourbarController.fourbarStatus.INTER;
                    if (timer_park.seconds()>29.5)
                    {
                        clawController.CS = clawController.clawStatus.OPENED;
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

            rightAutoController.update(r,clawAngle, claw, collectAngle, extendo, fourbar, lift);

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
            telemetry.addData("systems", RightAutoController.CurrentStatus);
            telemetry.addData("systems2", RightAutoController.CurrentStatus2);
            //  telemetry.addData("pwoer", r.collect.getPower());
            // telemetry.addData("nr cicluri scorare", nrcicluriscorare);
            //telemetry.addData("timer score", score_timer);
            // telemetry.addData("systems2", LeftAutoController.CurrentStatus2);
            //
            //telemetry.addData("globals i ", org.firstinspires.ftc.teamcode.Globals.globals.extendo_auto_i);
            //telemetry.addData("timer spit",timer_spit.seconds());
            // telemetry.addData("extendopos", extendopos);
            //telemetry.addData("liftpos", liftpos);
            telemetry.update();

            loopTime = loop;
            drive.update();
            telemetryA.update();


            poseUpdater.update();
            dashboardPoseTracker.update();

            Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
            Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
            Drawing.sendPacket();
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

    public boolean HasSpecimen(DigitalChannel distanceSensor)
    {
        return !distanceSensor.getState();
    }

}