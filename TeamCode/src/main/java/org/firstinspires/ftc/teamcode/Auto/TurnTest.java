package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Globals.robotMap;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.system_controllers.clawAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.clawController;
import org.firstinspires.ftc.teamcode.system_controllers.collectAngleController;
import org.firstinspires.ftc.teamcode.system_controllers.extendoController;
import org.firstinspires.ftc.teamcode.system_controllers.fourbarController;
import org.firstinspires.ftc.teamcode.system_controllers.liftController;
import org.firstinspires.ftc.teamcode.system_controllers.outtakeController;
import org.firstinspires.ftc.teamcode.system_controllers.transferController;

@Config
@Autonomous(group = "Auto", name = "TurnTest")
public class TurnTest extends LinearOpMode {
    enum STROBOT {
        START,
        PATH1,
        PATH2,
    }

    public Telemetry telemetryA;
    public static double scoreAngle = Math.toRadians(0);
    public static double color1CollectAngle = Math.toRadians(315);
    public static double color2CollectAngle = Math.toRadians(315);
    public static double color3CollectAngle = Math.toRadians(315);
    public static double color1SpitAngle = Math.toRadians(240);
    public static double color2SpitAngle = Math.toRadians(240);
    public static double color3SpitAngle = Math.toRadians(240);
    public static double specimenCollectAngle = Math.toRadians(0);
    public static double parkAngle = Math.toRadians(0);

    public PathChain pula;
    public
    boolean ok = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Follower drive = new Follower(hardwareMap);
        robotMap r = new robotMap(hardwareMap);

        int liftpos;
        int extendopos;
        /**
         * POSES
         */

        Pose startPose = new Pose(2,2,Math.toRadians(0));

        drive.setStartingPose(startPose);

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

        Pose preloadScorePose = new Pose(41,74,scoreAngle);
        Pose color1CollectPose = new Pose (33,37,color1CollectAngle);
        Pose color2CollectPose = new Pose (35,20,color2CollectAngle);
        Pose color3CollectPose = new Pose(35,7,color3CollectAngle);
        Pose color1SpitPose = new Pose(23,30,color1SpitAngle);
        Pose color2SpitPose = new Pose(23,12,color2SpitAngle);
        Pose color3SpitPose = new Pose(20,7,color3SpitAngle);
        Pose specimenCollectPose = new Pose(11,35,specimenCollectAngle);
        Pose specimen1ScorePose = new Pose(41,72,scoreAngle);
        Pose specimen2ScorePose = new Pose(42,70,scoreAngle);
        Pose specimen3ScorePose = new Pose(42,68,scoreAngle);
        Pose specimen4ScorePose = new Pose(42,66,scoreAngle);

        Pose parkPose = new Pose(11,30,parkAngle);



        /**
         * PATHS
         */

        Path path1 = new Path(new BezierCurve(new Point(0,0, Point.CARTESIAN), new Point(Math.abs(1),0, Point.CARTESIAN), new Point(Math.abs(1),1, Point.CARTESIAN)));
        path1.setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(180));



        pula = drive.pathBuilder()
                .addPath(path1)
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

        org.firstinspires.ftc.teamcode.Globals.globals.extendo_auto_i = 0;
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
           switch (status) {
               case START: {
                   drive.setStartingPose(startPose);
                   status = STROBOT.PATH1;
                   break;
               }

               case PATH1: {
                   drive.holdPoint(new BezierPoint(new Point(2,0,Point.CARTESIAN)),Math.toRadians(90));
                   status = STROBOT.PATH2;
                   break;
               }

               case PATH2:
               {
                   //drive.followPath(path2);
                   //status = RightAuto.PATH3;
                   break;
               }
           }
            liftpos = r.collect.getCurrentPosition();
            extendopos = r.extendo.getCurrentPosition();

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
            telemetry.addData("nr cicluri scorare", nrcicluriscorare);
            telemetry.addData("timer score", score_timer);
            // telemetry.addData("systems2", LeftAutoController.CurrentStatus2);
            //
            //telemetry.addData("globals i ", org.firstinspires.ftc.teamcode.Globals.globals.extendo_auto_i);
            //telemetry.addData("timer spit",timer_spit.seconds());
            telemetry.addData("extendopos", extendopos);
            telemetry.addData("liftpos", liftpos);
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