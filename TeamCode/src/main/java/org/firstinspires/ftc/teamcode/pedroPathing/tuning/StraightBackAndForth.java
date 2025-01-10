//package org.firstinspires.ftc.teamcode.pedroPathing.tuning;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
//import org.firstinspires.ftc.teamcode.pedroPathing.util.DashboardPoseTracker;
//
///**
// * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
// * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
// * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
// * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
// * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
// * have issues going in curves.
// *
// * @author Anyi Lin - 10158 Scott's Bots
// * @author Aaron Yang - 10158 Scott's Bots
// * @author Harrison Womack - 10158 Scott's Bots
// * @version 1.0, 3/12/2024
// */
//@Config
//@Autonomous (name = "Straight Back And Forth", group = "Autonomous Pathing Tuning")
//public class StraightBackAndForth extends OpMode {
//    private Telemetry telemetryA;
//
//    public static double DISTANCE = 40;
//
//    private boolean forward = true;
//
//    private Follower follower;
//
//    private Path forwards;
//    private Path backwards;
//    private PoseUpdater poseUpdater;
//    private DashboardPoseTracker dashboardPoseTracker;
//    public static double scoreAngle = Math.toRadians(0);
//    public static double color1CollectAngle = Math.toRadians(323);
//    public static double color2CollectAngle = Math.toRadians(315);
//    public static double color3CollectAngle = Math.toRadians(315);
//    public static double color1SpitAngle = Math.toRadians(250);
//    public static double color2SpitAngle = Math.toRadians(240);
//    public static double color3SpitAngle = Math.toRadians(240);
//    public static double specimenCollectAngle = Math.toRadians(0);
//    public static double parkAngle = Math.toRadians(0);
//    Pose preloadScorePose = new Pose(40.5,70,scoreAngle);
//    Pose color1CollectPose = new Pose (30,33,color1CollectAngle);
//    Pose color2CollectPose = new Pose (29,28,color2CollectAngle);
//    Pose color3CollectPose = new Pose(30,17,color3CollectAngle);
//    Pose color1SpitPose = new Pose(31,37,color1SpitAngle);
//    Pose color2SpitPose = new Pose(32,26,color2SpitAngle);
//    Pose color3SpitPose = new Pose(32,16,color3SpitAngle);
//    Pose specimen1AlignPose = new Pose(25,32,specimenCollectAngle);
//    Pose specimen1CollectPose = new Pose(11,35,specimenCollectAngle);
//    Pose specimen2AlignPose = new Pose(27,42,specimenCollectAngle);//27 42
//    Pose specimen2CollectPose = new Pose(11,35,specimenCollectAngle);
//    Pose specimen3AlignPose = new Pose(27,42,specimenCollectAngle);//27 42
//    Pose specimen3CollectPose = new Pose(11,35,specimenCollectAngle);
//    Pose specimen4AlignPose = new Pose(27,42,specimenCollectAngle);
//    Pose specimen4CollectPose = new Pose(11,35,specimenCollectAngle);
//    Pose specimen1ScorePose = new Pose(41.7,70,scoreAngle);
//    Pose specimen2ScorePose = new Pose(41.7,69,scoreAngle);
//    Pose specimen3ScorePose = new Pose(41.7,68,scoreAngle);
//    Pose specimen4ScorePose = new Pose(41.7,67,scoreAngle);
//    Pose startPose = new Pose(8,64,Math.toRadians(0));
//    Pose parkPose = new Pose(11,33,parkAngle);
//
//    /**
//     * PATHS
//     */
//    Path preload = new Path(new BezierLine(new Point(startPose), new Point(preloadScorePose)));
//    preload.setConstantHeadingInterpolation(Math.toRadians(0));
//
//    Path color1collect = new Path((new BezierLine(new Point(preloadScorePose),new Point(color1CollectPose))));
//        color1collect.setLinearHeadingInterpolation(scoreAngle,color1CollectAngle);
//
//    Path color1spit = new Path((new BezierLine(new Point(color1CollectPose),new Point(color1SpitPose))));
//        color1spit.setLinearHeadingInterpolation(color1CollectPose.getHeading(),color1SpitPose.getHeading());
//
//    Path color2collect = new Path((new BezierLine(new Point(color1SpitPose),new Point(color2CollectPose))));
//        color2collect.setLinearHeadingInterpolation(color1SpitPose.getHeading(),color2CollectPose.getHeading(),0);
//
//    Path color2spit = new Path((new BezierLine(new Point(color2CollectPose),new Point(color2SpitPose))));
//        color2spit.setLinearHeadingInterpolation(color2CollectPose.getHeading(),color2SpitPose.getHeading(),0);
//
//    Path color3collect = new Path((new BezierLine(new Point(color2SpitPose),new Point(color3CollectPose))));
//        color3collect.setLinearHeadingInterpolation(color2SpitAngle,color3CollectAngle, 0);
//
//    Path color3spit = new Path((new BezierLine(new Point(color3CollectPose),new Point(color3SpitPose))));
//        color3spit.setLinearHeadingInterpolation(color3CollectAngle,color3SpitAngle,0);
//
//    Path specimen1Align = new Path(new BezierLine(new Point(color3SpitPose),new Point(specimen1AlignPose)));
//        specimen1Align.setLinearHeadingInterpolation(color3SpitAngle,specimenCollectAngle,0);
//
//    Path specimen1Collect = new Path(new BezierLine(new Point(specimen1AlignPose),new Point(specimen1CollectPose)));
//        specimen1Collect.setConstantHeadingInterpolation(specimenCollectAngle);
//
//    Path specimen1Score = new Path(new BezierLine(new Point(specimen1CollectPose),new Point(specimen1ScorePose)));
//        specimen1Score.setConstantHeadingInterpolation(specimenCollectAngle);
//
//    Path specimen2Align = new Path(new BezierLine(new Point(specimen1ScorePose),new Point(specimen2AlignPose)));
//        specimen2Align.setConstantHeadingInterpolation(specimenCollectAngle);
//
//    Path specimen2Collect = new Path(new BezierLine(new Point(specimen2AlignPose),new Point(specimen2CollectPose)));
//        specimen2Collect.setConstantHeadingInterpolation(specimenCollectAngle);
//
//    Path specimen2Score = new Path(new BezierLine(new Point(specimen2CollectPose),new Point(specimen2ScorePose)));
//        specimen2Score.setConstantHeadingInterpolation(specimenCollectAngle);
//
//    Path specimen3Align = new Path(new BezierLine(new Point(specimen2ScorePose),new Point(specimen3AlignPose)));
//        specimen3Align.setConstantHeadingInterpolation(specimenCollectAngle);
//
//    Path specimen3Collect = new Path(new BezierLine(new Point(specimen3AlignPose),new Point(specimen3CollectPose)));
//        specimen3Collect.setConstantHeadingInterpolation(specimenCollectAngle);
//
//    Path specimen3Score = new Path(new BezierLine(new Point(specimen3CollectPose),new Point(specimen3ScorePose)));
//        specimen3Score.setConstantHeadingInterpolation(specimenCollectAngle);
//
//    Path specimen4Align = new Path(new BezierLine(new Point(specimen3ScorePose),new Point(specimen4AlignPose)));
//        specimen4Align.setConstantHeadingInterpolation(specimenCollectAngle);
//
//    Path specimen4Collect = new Path(new BezierLine(new Point(specimen4AlignPose),new Point(specimen4CollectPose)));
//        specimen4Collect.setConstantHeadingInterpolation(specimenCollectAngle);
//
//    Path specimen4Score = new Path(new BezierLine(new Point(specimen4CollectPose),new Point(specimen4ScorePose)));
//        specimen4Score.setConstantHeadingInterpolation(specimenCollectAngle);
//
//    Path park = new Path(new BezierLine(new Point(specimen4ScorePose),new Point(parkPose)));
//        park.setLinearHeadingInterpolation(scoreAngle,parkAngle) ;
//
//    Path specimen2curva = new Path(new BezierCurve(new Point(specimen1ScorePose),new Point(specimen2AlignPose),new Point(specimen2CollectPose)));
//        specimen2curva.setLinearHeadingInterpolation(0,0);
//
//    Path specimen3curva = new Path(new BezierCurve(new Point(specimen2ScorePose),new Point(specimen3AlignPose),new Point(specimen3CollectPose)));
//        specimen3curva.setLinearHeadingInterpolation(0,0);
//
//    Path specimen4curva = new Path(new BezierCurve(new Point(specimen3ScorePose),new Point(specimen4AlignPose),new Point(specimen4CollectPose)));
//        specimen4curva.setLinearHeadingInterpolation(0,0);
//
//    /**
//     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
//     * initializes the FTC Dashboard telemetry.
//     */
//    @Override
//    public void init() {
//        follower = new Follower(hardwareMap);
//
//        forwards =  new Path(new BezierLine(new Point(0,0,Point.CARTESIAN), new Point(0,20,Point.CARTESIAN)));
//        forwards.setLinearHeadingInterpolation(0, Math.toRadians(0));
//
//        backwards = new Path(new BezierLine(new Point(0,20, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
//        backwards.setLinearHeadingInterpolation((Math.toRadians(0)), 0);
//
//        follower.followPath(forwards);
//
//        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
//                            + " inches forward. The robot will go forward and backward continuously"
//                            + " along the path. Make sure you have enough room.");
//        telemetryA.update();
//    }
//
//    /**
//     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
//     * the Telemetry, as well as the FTC Dashboard.
//     */
//    @Override
//    public void loop() {
//        follower.update();
//        if (!follower.isBusy()) {
//            if (forward) {
//                forward = false;
//                follower.followPath(backwards);
//            } else {
//                forward = true;
//                follower.followPath(forwards);
//            }
//        }
//
//        telemetryA.addData("going forward", forward);
//        follower.telemetryDebug(telemetryA);
//    }
//}
