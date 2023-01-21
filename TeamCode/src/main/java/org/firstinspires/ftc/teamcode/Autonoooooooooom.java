package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner.COLOR_INACTIVE_TRAJECTORY;
import static org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner.COLOR_INACTIVE_TURN;
import static org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner.COLOR_INACTIVE_WAIT;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.openftc.easyopencv.OpenCvCamera;

import java.time.chrono.ThaiBuddhistEra;
import java.util.Vector;

@Config
@Autonomous(group = "drive")
@SuppressLint("DefaultLocale")
public class Autonoooooooooom extends LinearOpMode {

    private final FtcDashboard dashboard;

    OpenCvCamera webcam;
    AprilTagDetectionPipeline pipeline;

    SampleMecanumDrive drive;

    public Autonoooooooooom() {
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
    }

    int ERROR = 0;

    void sError(int err) {
        ERROR = err;
    }

    final double TAGSIZE = 4.5 / 100;
    final double FX = 878.272;
    final double FY = 878.272;
    final double CX = 320;
    final double CY = 240;

    final double PI2 = Math.PI / 2;
    final double PI = Math.PI;

    int LAST_ID = 0;

    public int TOP_POS = 1220;
    public int MIU_POS = 1010;
    public int MID_POS = 760;

    ThaiBuddhistEra thaiBuddhistEra; // 777hz tibetan healing sounds

    public static int F = 65;

    double S1CL = 0.362;
    double S1OP = 0.58;

    public static double HEAD1 = Math.toRadians(54.56);
    public static double PX1 = 151;
    public static double PY1 = -1.4;
    public static double HEAD2 = Math.toRadians(270);
    public static double PX2 = 135;
    public static double PY2 = -50;
    public static double HEAD3 = Math.toRadians(46);
    public static double PX3 = 155.5;
    public static double PY3 = -0.5;
    public static double HEADC = Math.toRadians(-1.3);
    public static double PXC = 1.8;
    public static double PYC = 1.7;

    public static double P1X = 30;
    public static double P1Y = Math.PI;
    public static double P2X = 10;
    public static double P2Y = Math.PI / 2;

    public static boolean AAAAAAAAAAAAAA = false;
    public static boolean BBBBBBBBBBBBBB = true;
    public static boolean RECURRING_SINGULARITY = true;

    public static double MVEL = 165;
    public static double MAL = 165;

    public static double OPD = 0.3;
    public static double UPD = 0.7;
    public static double WD = 0.2;

    public static double R1X = 20;
    public static double R1Y = 0.0;
    public static double R2X = 50;
    public static double R2Y = Math.PI * 1.25;

    VoltageSensor batteryVoltageSensor;

    Vector<Double> v = new Vector<>();
    double it;

    void ltime() {
        v.add(getRuntime() - it);
    }

    void follow_traj(TrajectorySequence traj) {
        drive.followTrajectorySequenceAsync(traj);
        drive.update();
        while (drive.isBusy() && !isStopRequested() && traj != null) {
            /*telemetry.speak("Buna ziua!");
            telemetry.addData("Traj", "Going from (%f, %f, %f) to (%f, %f, %f) for %f", traj.start().getX(), traj.start().getY(), traj.start().getHeading(),
                    traj.end().getX(), traj.end().getY(), traj.end().getHeading(),
                    traj.duration());
            telemetry.addData("Normal", NORMAL);*/
            drive.update();
        }
    }

    private void draw(
            Canvas fieldOverlay,
            TrajectorySequence sequence
    ) {
        if (sequence != null) {
            for (int i = 0; i < sequence.size(); i++) {
                SequenceSegment segment = sequence.get(i);

                if (segment instanceof TrajectorySegment) {
                    fieldOverlay.setStrokeWidth(1);
                    fieldOverlay.setStroke(COLOR_INACTIVE_TRAJECTORY);

                    DashboardUtil.drawSampledPath(fieldOverlay, ((TrajectorySegment) segment).getTrajectory().getPath());
                } else if (segment instanceof TurnSegment) {
                    Pose2d pose = segment.getStartPose();

                    fieldOverlay.setFill(COLOR_INACTIVE_TURN);
                    fieldOverlay.fillCircle(pose.getX(), pose.getY(), 2);
                } else if (segment instanceof WaitSegment) {
                    Pose2d pose = segment.getStartPose();

                    fieldOverlay.setStrokeWidth(1);
                    fieldOverlay.setStroke(COLOR_INACTIVE_WAIT);
                    fieldOverlay.strokeCircle(pose.getX(), pose.getY(), 3);
                }
            }
        }
    }

    TrajectorySequence mktraj(DcMotor ridicareSlide, Servo s1) {
        Vector2d P1 = new Vector2d(P1X, P1Y);
        Vector2d P2 = new Vector2d(P2X, P2Y);
        Vector2d R1 = new Vector2d(R1X, R1Y);
        Vector2d R2 = new Vector2d(R2X, R2Y);
        TrajectoryVelocityConstraint vc = SampleMecanumDrive.getVelocityConstraint(MVEL, MAX_ANG_VEL, TRACK_WIDTH);
        TrajectoryAccelerationConstraint ac = SampleMecanumDrive.getAccelerationConstraint(MAL);
        return drive.trajectorySequenceBuilder(new Pose2d())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    ThreadInfo.target = 100;
                    ridicareSlide.setPower(0.4);
                    ridicareSlide.setTargetPosition(100);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    ThreadInfo.target = TOP_POS;
                    ridicareSlide.setPower(1.2);
                    ridicareSlide.setTargetPosition(TOP_POS);
                    s1.setPosition(S1CL);
                })
                .funnyRaikuCurve(new Pose2d(PX1, PY1, HEAD1), R1, R2, vc, ac)
                .addTemporalMarker(this::ltime)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    ThreadInfo.target = TOP_POS - 200;
                    ridicareSlide.setTargetPosition(TOP_POS - 200);
                })
                .waitSeconds(WD)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> s1.setPosition(S1OP)) ///////////////////////////// PRELOAD 1
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    ThreadInfo.target = 360;
                    ridicareSlide.setPower(0.5);
                    ridicareSlide.setTargetPosition(360);
                })
                .funnyRaikuCurve(new Pose2d(PX2, PY2, HEAD2), new Vector2d(P1.getX() * 2, P1.getY()), P2) //////////////////////////////////////////// GET CONE 1
                .addTemporalMarker(this::ltime)
                .UNSTABLE_addTemporalMarkerOffset(-0.03, () -> s1.setPosition(S1CL))
                .waitSeconds(0.12)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    ThreadInfo.target = TOP_POS / 2;
                    ridicareSlide.setPower(1);
                    ridicareSlide.setTargetPosition(TOP_POS / 2);
                })
                .UNSTABLE_addTemporalMarkerOffset(UPD, () -> {
                    ThreadInfo.target = TOP_POS;
                    ridicareSlide.setTargetPosition(TOP_POS);
                })
                .funnyRaikuCurve(new Pose2d(PX3, PY3, HEAD3), P2, P1)
                .addTemporalMarker(this::ltime)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    ThreadInfo.target = TOP_POS - 200;
                    ridicareSlide.setTargetPosition(TOP_POS - 200);
                })
                .UNSTABLE_addTemporalMarkerOffset(OPD, () -> s1.setPosition(S1OP)) ///////////////////////////// CONE 1
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    ThreadInfo.target = 310;
                    ridicareSlide.setPower(0.4);
                    ridicareSlide.setTargetPosition(310);
                })
                .funnyRaikuCurve(new Pose2d(PX2 + PXC, PY2, HEAD2), P1, P2) //////////////////////////////////////////// GET CONE 2
                .addTemporalMarker(this::ltime)
                .UNSTABLE_addTemporalMarkerOffset(-0.03, () -> s1.setPosition(S1CL))
                .waitSeconds(0.12)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    ThreadInfo.target = TOP_POS / 2;
                    ridicareSlide.setPower(1);
                    ridicareSlide.setTargetPosition(TOP_POS / 2);
                })
                .UNSTABLE_addTemporalMarkerOffset(UPD, () -> {
                    ThreadInfo.target = TOP_POS;
                    ridicareSlide.setTargetPosition(TOP_POS);
                })
                .funnyRaikuCurve(new Pose2d(PX3 + PXC * 1.25, PY3 + PYC, HEAD3 + HEADC), P2, P1)
                .addTemporalMarker(this::ltime)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    ThreadInfo.target = TOP_POS - 200;
                    ridicareSlide.setTargetPosition(TOP_POS - 200);
                })
                .UNSTABLE_addTemporalMarkerOffset(OPD, () -> s1.setPosition(S1OP)) ///////////////////////////// CONE 2
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    ThreadInfo.target = 260;
                    ridicareSlide.setPower(0.4);
                    ridicareSlide.setTargetPosition(260);
                })
                .funnyRaikuCurve(new Pose2d(PX2 + PXC * 2, PY2, HEAD2), P1, P2) //////////////////////////////////////////// GET CONE 3
                .addTemporalMarker(this::ltime)
                .UNSTABLE_addTemporalMarkerOffset(-0.03, () -> s1.setPosition(S1CL))
                .waitSeconds(0.12)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    ThreadInfo.target = TOP_POS / 2;
                    ridicareSlide.setPower(1);
                    ridicareSlide.setTargetPosition(TOP_POS / 2);
                })
                .UNSTABLE_addTemporalMarkerOffset(UPD, () -> {
                    ThreadInfo.target = TOP_POS;
                    ridicareSlide.setTargetPosition(TOP_POS);
                })
                .funnyRaikuCurve(new Pose2d(PX3 + PXC * 2.5, PY3 + PYC * 2, HEAD3 + HEADC * 2.4), P2, P1)
                .addTemporalMarker(this::ltime)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    ThreadInfo.target = TOP_POS - 200;
                    ridicareSlide.setTargetPosition(TOP_POS - 200);
                })
                .UNSTABLE_addTemporalMarkerOffset(OPD, () -> s1.setPosition(S1OP)) ///////////////////////////// CONE 3
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    ThreadInfo.target = 210;
                    ridicareSlide.setPower(0.4);
                    ridicareSlide.setTargetPosition(210);
                })
                .funnyRaikuCurve(new Pose2d(PX2 + PXC * 3.75, PY2, HEAD2), P1, P2) //////////////////////////////////////////// GET CONE 4
                .addTemporalMarker(this::ltime)
                .UNSTABLE_addTemporalMarkerOffset(-0.03, () -> s1.setPosition(S1CL))
                .waitSeconds(0.12)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    ThreadInfo.target = TOP_POS / 2;
                    ridicareSlide.setPower(1);
                    ridicareSlide.setTargetPosition(TOP_POS / 2);
                })
                .UNSTABLE_addTemporalMarkerOffset(UPD, () -> {
                    ThreadInfo.target = TOP_POS;
                    ridicareSlide.setTargetPosition(TOP_POS);
                })
                .funnyRaikuCurve(new Pose2d(PX3 + PXC * 3.1, PY3 + PYC * 3, HEAD3 + HEADC * 4.5), P2, P1)
                .addTemporalMarker(this::ltime)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    ThreadInfo.target = TOP_POS - 200;
                    ridicareSlide.setTargetPosition(TOP_POS - 200);
                })
                .UNSTABLE_addTemporalMarkerOffset(OPD, () -> s1.setPosition(S1OP)) ///////////////////////////// CONE 4
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    ThreadInfo.target = 70;
                    ridicareSlide.setPower(0.4);
                    ridicareSlide.setTargetPosition(70);
                })
                .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Servo s1 = hardwareMap.get(Servo.class, "S1");
        DcMotorEx ridicareSlide = hardwareMap.get(DcMotorEx.class, "RS");
        ridicareSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ridicareSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        ridicareSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        s1.setPosition(S1CL);

        /*Runnable armRun = new ArmcPIDF(ridicareSlide);
        Thread armThread = new Thread(armRun);*/

        drive = new SampleMecanumDrive(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        TelemetryPacket packet;

        /*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new AprilTagDetectionPipeline(TAGSIZE, FX, FY, CX, CY);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
                dashboard.startCameraStream(webcam, 15);
                OPENED = true;
            }

            @Override
            public void onError(int errorCode) {
                sError(errorCode);
            }
        });

        while (!isStarted() && !isStopRequested()) {
            if (OPENED) {
                if (LAST_ID != 0) {
                    telemetry.addLine("Cam opened");
                    telemetry.update();
                }
                ArrayList<AprilTagDetection> cd = pipeline.getLatestDetections();
                if (cd.size() > 0) {
                    LAST_ID = cd.get(0).id;
                    packet = new TelemetryPacket();
                    packet.put("LID", LAST_ID);
                    dashboard.sendTelemetryPacket(packet);
                    telemetry.addData("Got id: ", LAST_ID);
                    telemetry.update();
                }
            } else {
                telemetry.addLine("Waiting on cam open");
                telemetry.update();
            }
            sleep(100);
        }*/

        waitForStart();

        /*ThreadInfo.shouldClose = false;
        armThread.start();
        ThreadInfo.use = true;
        ThreadInfo.target = 40;*/


        /*ridicareSlide.setPower(0.5);
        ridicareSlide.setTargetPosition(40);
        ridicareSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
        //webcam.closeCameraDeviceAsync(() -> {});

        TelemetryPacket pack = new TelemetryPacket();
        pack.put("bat", batteryVoltageSensor.getVoltage());
        dashboard.sendTelemetryPacket(pack);

        TrajectorySequence traj = null;

        if (!BBBBBBBBBBBBBB) {
            telemetry.addLine("Start");
            telemetry.update();
            double P11X = 0, P11Y = 0, P12X = 0, P12Y = 0;

            while (!isStopRequested()) {
                if (P11X != PX1 || P12X != PX2 || P11Y != PY1 || P12Y != PY2) {
                    traj = mktraj(ridicareSlide, s1);
                    telemetry.addLine("ct");
                    telemetry.update();
                    TelemetryPacket p = new TelemetryPacket();
                    Canvas fieldOverlay = p.fieldOverlay();
                    telemetry.addLine("gov");
                    telemetry.update();
                    draw(fieldOverlay, traj);
                    telemetry.addLine("draw");
                    telemetry.update();
                    p.put("Updated!", 0);
                    p.put("StartX", traj.start().getX());
                    p.put("StartY", traj.start().getY());
                    p.put("StartH", traj.start().getY());
                    p.put("EndX", traj.start().getX());
                    p.put("EndY", traj.start().getY());
                    p.put("EndH", traj.start().getY());
                    telemetry.addLine("gett");
                    telemetry.update();
                    dashboard.sendTelemetryPacket(p);
                    P11X = P1X;
                    P11Y = P1Y;
                    P12X = P2X;
                    P12Y = P2Y;
                }

                sleep(100);
            }
        } else {
            if (AAAAAAAAAAAAAA) {
                while (!isStopRequested()) {
                    if (!drive.isBusy()) {
                        traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(0, -F, 0))
                                .lineToLinearHeading(new Pose2d(0, F, 0))
                                .build();
                        drive.followTrajectorySequenceAsync(traj);
                    }
                    drive.update();
                }

                //telemetry stuff vezi unde esti
                while (!isStopRequested()) {
                    drive.updatePoseEstimate();
                    packet = new TelemetryPacket();
                    packet.put("px", drive.getPoseEstimate().getX());
                    packet.put("py", drive.getPoseEstimate().getY());
                    packet.put("ph", drive.getPoseEstimate().getHeading());
                    dashboard.sendTelemetryPacket(packet);
                }
            } else {
                packet = new TelemetryPacket();
                packet.put("LID", LAST_ID);
                dashboard.sendTelemetryPacket(packet);

                traj = mktraj(ridicareSlide, s1);

                it = getRuntime();
                follow_traj(traj);

                pack = new TelemetryPacket();
                for (int i = 0; i < v.size(); ++i) {
                    pack.put("TIME" + i, v.get(i));
                }
                dashboard.sendTelemetryPacket(pack);

                switch (LAST_ID) {
                    default:
                    case 6:
                        traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(F * 2, 0, 0))
                                .build();
                        break;
                    case 7:
                        traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(F * 2, F, 0))
                                .build();
                        break;
                    case 8:
                        traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(F * 2, -F, 0))
                                .build();
                        break;
                }
                follow_traj(traj);
            }

            if (RECURRING_SINGULARITY) {
                traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(15, 0, 0))
                        .build();
                follow_traj(traj);
            }
        }

        /*try {
            ThreadInfo.shouldClose = true;
            armThread.join();
        } catch (Exception e) {
            e.printStackTrace();
        }*/
    }
}
