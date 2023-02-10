package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.SDESCHIS;
import static org.firstinspires.ftc.teamcode.RobotConstants.SINCHIS;
import static org.firstinspires.ftc.teamcode.RobotConstants.TOP_POS;
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
import com.acmerobotics.roadrunner.trajectory.DisplacementProducer;
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.time.chrono.ThaiBuddhistEra;
import java.util.ArrayList;
import java.util.Vector;

@SuppressWarnings("CommentedOutCode")
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

    int LAST_ID = 0;

    ThaiBuddhistEra thaiBuddhistEra; // 777hz tibetan healing sounds

    public int F = 65;

    public static double HEAD1 = 0.69;
    public static double PX1 = 148.77;
    public static double PY1 = 12;
    public static double HEAD2 = Math.toRadians(270);
    public static double PX2 = 141;
    public static double PY2 = -47.5;
    public static double HEAD3 = 0.83;
    public static double PX3 = PX1;
    public static double PY3 = 12;
    public static double HEADC = 0.065;
    public static double PXC = 0.3;
    public static double PXXC = 1.3;
    public static double PYC = 1.5;
    public static double PCY = 0.5;

    public static double P1X = 43;
    public static double P1Y = 3.7;
    public static double P2X = 50;
    public static double P2Y = 1.9;

    public static boolean AAAAAAAAAAAAAA = false;
    public static boolean BBBBBBBBBBBBBB = true;
    public static boolean RECURRING_SINGULARITY = true;
    public static boolean GPOS = false;

    public static double MVEL = 150;//120;
    public static double MAL = 100;//120;
    public static double MDL = 80;//70;

    public double OPD = 0.04;
    public double UPD = 0.7;
    public double WTD = 0.3;
    public double WD = 0.02;
    public double WWD = 0.04;
    public static double PD = 0.3;

    public static double R1X = 25;
    public static double R1Y = 0.1;
    public static double R2X = 35;
    public static double R2Y = 3.8;

    public double H11 = -0.8;
    public double H12 = 0.8;
    public double H21 = -1;
    public double H22 = 1;

    int GP1 = 280;
    int GP2 = 250;
    int GP3 = 230;
    int GP4 = 130;
    int GP5 = 100;

    VoltageSensor batteryVoltageSensor;
    public static boolean CYCLE = false;

    Vector<Double> v = new Vector<>();
    Vector<Pose2d> e = new Vector<>();
    double it;

    void ltime() {
        v.add(getRuntime() - it);
        e.add(drive.getLastError());
    }

    void follow_traj(TrajectorySequence traj) {
        drive.followTrajectorySequenceAsync(traj);
        drive.update();
        ElapsedTime timer = new ElapsedTime(0);
        while (drive.isBusy() && !isStopRequested() && traj != null) {
            if (CYCLE) {
                TelemetryPacket pack = new TelemetryPacket();
                pack.put("CycleTime", timer.milliseconds());
                dashboard.sendTelemetryPacket(pack);
            }
            drive.update();
        }
    }

    private void draw(
            Canvas fieldOverlay,
            TrajectorySequence sequence
    ) {
        double ITC = 1 / 2.54;
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
                    fieldOverlay.fillCircle(pose.getX() * ITC, pose.getY() * ITC, 2);
                } else if (segment instanceof WaitSegment) {
                    Pose2d pose = segment.getStartPose();

                    fieldOverlay.setStrokeWidth(1);
                    fieldOverlay.setStroke(COLOR_INACTIVE_WAIT);
                    fieldOverlay.strokeCircle(pose.getX() * ITC, pose.getY() * ITC, 3);
                }
            }
        }
    }

    @SuppressWarnings("ConstantConditions")
    TrajectorySequence mktraj(DcMotor ridicareSlide, Servo s1) {
        if (false) {
            TrajectoryVelocityConstraint vc = SampleMecanumDrive.getVelocityConstraint(MVEL, MAX_ANG_VEL, TRACK_WIDTH);
            TrajectoryAccelerationConstraint ac = SampleMecanumDrive.getAccelerationConstraint(MAL);
            return drive.trajectorySequenceBuilder(new Pose2d())
                    .lineToConstantHeading(new Vector2d(F * 2, 0), vc, ac)
                    //.lineToConstantHeading(new Vector2d(F * 2, 0))
                    .build();
        } else {
            Vector2d P1 = new Vector2d(P1X, P1Y);
            Vector2d P2 = new Vector2d(P2X, P2Y);
            Vector2d R1 = new Vector2d(R1X, R1Y);
            Vector2d R2 = new Vector2d(R2X, R2Y);
            TrajectoryVelocityConstraint vc = SampleMecanumDrive.getVelocityConstraint(MVEL, MAX_ANG_VEL, TRACK_WIDTH);
            TrajectoryAccelerationConstraint ac = SampleMecanumDrive.getAccelerationConstraint(MAL);
            TrajectoryAccelerationConstraint dc = SampleMecanumDrive.getAccelerationConstraint(MDL);
            return drive.trajectorySequenceBuilder(new Pose2d(SPOSX, SPOSY, SPOSH))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        ThreadInfo.target = 100;
                        ridicareSlide.setPower(0.4);
                        ridicareSlide.setTargetPosition(100);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(WTD, () -> {
                        ThreadInfo.target = TOP_POS;
                        ridicareSlide.setPower(1.2);
                        ridicareSlide.setTargetPosition(TOP_POS);
                        s1.setPosition(SINCHIS);
                    })
                    .funnyRaikuCurve(new Pose2d(PX1, PY1, HEAD1), R1, R2, H11, H12, vc, ac, dc)
                    .addTemporalMarker(this::ltime)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        getpos();
                        ThreadInfo.target = TOP_POS - 300;
                        ridicareSlide.setTargetPosition(TOP_POS - 250);
                    })
                    .waitSeconds(WD)
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s1.setPosition(SDESCHIS)) ///////////////////////////// PRELOAD 1
                    .UNSTABLE_addTemporalMarkerOffset(PD, () -> {
                        ThreadInfo.target = 375;
                        ridicareSlide.setPower(0.5);
                        ridicareSlide.setTargetPosition(GP1);
                    })
                    .funnyRaikuCurve(new Pose2d(PX2, PY2, HEAD2), P1, P2, H21, H22) //////////////////////////////////////////// GET CONE 1
                    .addTemporalMarker(this::ltime)
                    .UNSTABLE_addTemporalMarkerOffset(-0.03, () -> {
                        getpos();
                        s1.setPosition(SINCHIS);
                    })
                    .waitSeconds(WWD)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        ThreadInfo.target = TOP_POS / 2;
                        ridicareSlide.setPower(1);
                        ridicareSlide.setTargetPosition(TOP_POS / 2);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(UPD, () -> {
                        ThreadInfo.target = TOP_POS;
                        ridicareSlide.setTargetPosition(TOP_POS);
                    })
                    .funnyRaikuCurve(new Pose2d(PX3, PY3, HEAD3), P2, P1, H21, H22)
                    .addTemporalMarker(this::ltime)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        getpos();
                        ThreadInfo.target = TOP_POS - 200;
                        ridicareSlide.setTargetPosition(TOP_POS - 250);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(OPD, () -> s1.setPosition(SDESCHIS)) ///////////////////////////// CONE 1
                    .UNSTABLE_addTemporalMarkerOffset(PD, () -> {
                        ThreadInfo.target = 310;
                        ridicareSlide.setPower(0.5);
                        ridicareSlide.setTargetPosition(GP2);
                    })
                    .funnyRaikuCurve(new Pose2d(PX2 + PXXC, PY2 + PCY, HEAD2), P1, P2, H21, H22) //////////////////////////////////////////// GET CONE 2
                    .addTemporalMarker(this::ltime)
                    .UNSTABLE_addTemporalMarkerOffset(-0.03, () -> s1.setPosition(SINCHIS))
                    .waitSeconds(WWD)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        ThreadInfo.target = TOP_POS / 2;
                        ridicareSlide.setPower(1);
                        ridicareSlide.setTargetPosition(TOP_POS / 2);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(UPD, () -> {
                        ThreadInfo.target = TOP_POS;
                        ridicareSlide.setTargetPosition(TOP_POS);
                    })
                    .funnyRaikuCurve(new Pose2d(PX3 + PXC * 1, PY3 + PYC * 1, HEAD3 + HEADC), P2, P1, H21, H22)
                    .addTemporalMarker(this::ltime)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        getpos();
                        ThreadInfo.target = TOP_POS - 200;
                        ridicareSlide.setTargetPosition(TOP_POS - 250);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(OPD, () -> s1.setPosition(SDESCHIS)) ///////////////////////////// CONE 2
                    .UNSTABLE_addTemporalMarkerOffset(PD, () -> {
                        ThreadInfo.target = 260;
                        ridicareSlide.setPower(0.5);
                        ridicareSlide.setTargetPosition(GP3);
                    })
                    .funnyRaikuCurve(new Pose2d(PX2 + PXXC * 2, PY2 + PCY * 2, HEAD2), P1, P2, H21, H22) //////////////////////////////////////////// GET CONE 3
                    .addTemporalMarker(this::ltime)
                    .UNSTABLE_addTemporalMarkerOffset(-0.03, () -> s1.setPosition(SINCHIS))
                    .waitSeconds(WWD)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        ThreadInfo.target = TOP_POS / 2;
                        ridicareSlide.setPower(1);
                        ridicareSlide.setTargetPosition(TOP_POS / 2);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(UPD, () -> {
                        ThreadInfo.target = TOP_POS;
                        ridicareSlide.setTargetPosition(TOP_POS);
                    })
                    .funnyRaikuCurve(new Pose2d(PX3 + PXC * 2, PY3 + PYC * 2, HEAD3 + HEADC * 2), P2, P1, H21, H22)
                    .addTemporalMarker(this::ltime)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        getpos();
                        ThreadInfo.target = TOP_POS - 200;
                        ridicareSlide.setTargetPosition(TOP_POS - 250);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(OPD, () -> s1.setPosition(SDESCHIS)) ///////////////////////////// CONE 3
                    .UNSTABLE_addTemporalMarkerOffset(PD, () -> {
                        ThreadInfo.target = 210;
                        ridicareSlide.setPower(0.5);
                        ridicareSlide.setTargetPosition(GP4);
                    })
                    .funnyRaikuCurve(new Pose2d(PX2 + PXXC * 3, PY2 + PCY * 3, HEAD2), P1, P2, H21, H22) //////////////////////////////////////////// GET CONE 4
                    .addTemporalMarker(this::ltime)
                    .UNSTABLE_addTemporalMarkerOffset(-0.03, () -> s1.setPosition(SINCHIS))
                    .waitSeconds(WWD)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        ThreadInfo.target = TOP_POS / 2;
                        ridicareSlide.setPower(1);
                        ridicareSlide.setTargetPosition(TOP_POS / 2);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(UPD, () -> {
                        ThreadInfo.target = TOP_POS;
                        ridicareSlide.setTargetPosition(TOP_POS);
                    })
                    .funnyRaikuCurve(new Pose2d(PX3 + PXC * 3, PY3 + PYC * 3, HEAD3 + HEADC * 2.9), P2, P1, H21, H22)
                    .addTemporalMarker(this::ltime)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        getpos();
                        ThreadInfo.target = TOP_POS - 200;
                        ridicareSlide.setTargetPosition(TOP_POS - 250);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(OPD, () -> s1.setPosition(SDESCHIS)) ///////////////////////////// CONE 4
                    .UNSTABLE_addTemporalMarkerOffset(PD, () -> {
                        ThreadInfo.target = 210;
                        ridicareSlide.setPower(0.5);
                        ridicareSlide.setTargetPosition(GP5);
                    })
                    .funnyRaikuCurve(new Pose2d(PX2 + PXXC * 4, PY2 + PCY * 4, HEAD2), P1, P2, H21, H22) //////////////////////////////////////////// GET CONE 5
                    .addTemporalMarker(this::ltime)
                    .UNSTABLE_addTemporalMarkerOffset(-0.03, () -> s1.setPosition(SINCHIS))
                    .waitSeconds(WWD)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        ThreadInfo.target = TOP_POS / 2;
                        ridicareSlide.setPower(1);
                        ridicareSlide.setTargetPosition(TOP_POS / 2);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(UPD, () -> {
                        ThreadInfo.target = TOP_POS;
                        ridicareSlide.setTargetPosition(TOP_POS);
                    })
                    .funnyRaikuCurve(new Pose2d(PX3 + PXC * 4, PY3 + PYC * 4, HEAD3 + HEADC * 3.8), P2, P1, H21, H22)
                    .addTemporalMarker(this::ltime)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        getpos();
                        ThreadInfo.target = TOP_POS - 200;
                        ridicareSlide.setTargetPosition(TOP_POS - 250);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(OPD, () -> s1.setPosition(SDESCHIS)) ///////////////////////////// CONE 5
                    .waitSeconds(0.2)
                    .build();
        }
    }

    boolean OPENED = false;
    public static double SPOSX = 0;
    public static double SPOSY = 0;
    public static double SPOSH = 0;

    public DcMotorEx leftBack;
    public DcMotorEx leftFront;
    public DcMotorEx rightBack;
    public DcMotorEx rightFront;

    void getpos() {
        if (GPOS) {
            FtcDashboard dash = FtcDashboard.getInstance();
            while (!isStopRequested()) {
                drive.updatePoseEstimate();
                telemetry.addData("PE", drive.getPoseEstimate());
                telemetry.addData("Pe", drive.getLastError());
                telemetry.update();
                TelemetryPacket p = new TelemetryPacket();
                p.put("Rx", drive.getPoseEstimate().getX());
                p.put("Ry", drive.getPoseEstimate().getY());
                p.put("Rh", drive.getPoseEstimate().getHeading());
                p.put("Ex", drive.getLastError().getX());
                p.put("Ey", drive.getLastError().getY());
                p.put("Eh", drive.getLastError().getHeading());
                dash.sendTelemetryPacket(p);

                final double speed = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
                final double angle = Math.atan2(-gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
                final double turn = gamepad1.right_stick_x;
                final double ms = speed * Math.sin(angle);
                final double mc = speed * Math.cos(angle);
                //maths (nu stiu eu deastea ca fac cu antohe)
                final double lfPower = ms + turn;
                final double rfPower = mc - turn;
                final double lbPower = mc + turn;
                final double rbPower = ms - turn;
                final double pcoef = 12.0 / batteryVoltageSensor.getVoltage();
                final double spcoef = 1 - 0.6 * gamepad1.right_trigger;
                final double fcoef = pcoef * spcoef * 0.7;
                leftFront.setPower(lfPower * fcoef);
                rightFront.setPower(rfPower * fcoef);
                leftBack.setPower(lbPower * fcoef);
                rightBack.setPower(rbPower * fcoef);

                if (gamepad1.a) {
                    break;
                }
                if (gamepad1.b) {
                    ridicareSlide.setPower(0.4);
                    ridicareSlide.setTargetPosition(100);
                }
            }
        }
    }

    DcMotorEx ridicareSlide;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo s1 = hardwareMap.get(Servo.class, "S1");
        ridicareSlide = hardwareMap.get(DcMotorEx.class, "RS");
        ridicareSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ridicareSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        ridicareSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        s1.setPosition(SINCHIS);

        leftBack = hardwareMap.get(DcMotorEx.class, "LB");
        leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        rightBack = hardwareMap.get(DcMotorEx.class, "RB");
        rightFront = hardwareMap.get(DcMotorEx.class, "RF");
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.speak("Dami whynd bo-ooh-le!");

        /*Runnable armRun = new ArmcPIDF(ridicareSlide);
        Thread armThread = new Thread(armRun);*/

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(SPOSX, SPOSY, SPOSH));

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        TelemetryPacket packet;

        /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new AprilTagDetectionPipeline(TAGSIZE, FX, FY, CX, CY);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                if (!opModeIsActive()) {
                    webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
                    dashboard.startCameraStream(webcam, 15);
                    OPENED = true;
                }
            }

            @Override
            public void onError(int errorCode) {
                sError(errorCode);
            }
        });

        while (!isStarted() && !isStopRequested()) {
            if (OPENED) {
                if (LAST_ID == 0) {
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

        TrajectorySequence traj = null;

        if (BBBBBBBBBBBBBB) {
            traj = mktraj(ridicareSlide, s1);
        }
        waitForStart();

        /*ThreadInfo.shouldClose = false;
        armThread.start();
        ThreadInfo.use = true;
        ThreadInfo.target = 40;*/


        drive.setPoseEstimate(new Pose2d(SPOSX, SPOSY, SPOSH));
        ridicareSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ridicareSlide.setPower(0.5);
        ridicareSlide.setTargetPosition(40);
        ridicareSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (OPENED) {
            webcam.closeCameraDeviceAsync(() -> {
            });
        }

        TelemetryPacket pack = new TelemetryPacket();
        pack.put("bat", batteryVoltageSensor.getVoltage());
        dashboard.sendTelemetryPacket(pack);

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

                // traj = mktraj(ridicareSlide, s1);

                it = getRuntime();
                follow_traj(traj);

                TrajectoryVelocityConstraint vc = SampleMecanumDrive.getVelocityConstraint(MVEL, MAX_ANG_VEL, TRACK_WIDTH);
                TrajectoryAccelerationConstraint ac = SampleMecanumDrive.getAccelerationConstraint(MAL);
                /*if (!isStopRequested()) {
                    switch (LAST_ID) {
                        default:
                            telemetry.addLine("DEFAULT");
                        case 7:
                            telemetry.addLine("6");
                            traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    //.funnyRaikuCurve(new Pose2d(F * 2.15, 0, 0), new Vector2d(20, Math.PI), new Vector2d(0.00001, 0.0), vc, ac)
                                    .lineToLinearHeading(new Pose2d(F * 2, 0, 0))
                                    .addTemporalMarker(this::ltime)
                                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                                        ThreadInfo.target = 70;
                                        ridicareSlide.setPower(0.5);
                                        ridicareSlide.setTargetPosition(70);
                                    })
                                    .waitSeconds(1)
                                    .build();
                            break;
                        case 6:
                            telemetry.addLine("7");
                            traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    //.funnyRaikuCurve(new Pose2d(F * 2.15, F * 1, 0), new Vector2d(20, Math.PI), new Vector2d(0.00001, 0.0), vc, ac)
                                    .lineToLinearHeading(new Pose2d(F * 2, F, 0))
                                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                                        ThreadInfo.target = 70;
                                        ridicareSlide.setPower(0.5);
                                        ridicareSlide.setTargetPosition(70);
                                    })
                                    .addTemporalMarker(this::ltime)
                                    .waitSeconds(1)
                                    .build();
                            break;
                        case 8:
                            telemetry.addLine("8");
                            traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    //.funnyRaikuCurve(new Pose2d(F * 2.15, -F * 1, 0), new Vector2d(20, Math.PI), new Vector2d(0.00001, 0.0), vc, ac)
                                    .lineToLinearHeading(new Pose2d(F * 2, -F, 0))
                                    .addTemporalMarker(this::ltime)
                                    .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> {
                                        ThreadInfo.target = 70;
                                        ridicareSlide.setPower(0.5);
                                        ridicareSlide.setTargetPosition(70);
                                    })
                                    .waitSeconds(1)
                                    .build();
                            break;
                    }
                    telemetry.update();
                    follow_traj(traj);
                }*/
            }

            if (GPOS) {
                getpos();
            }

            if (RECURRING_SINGULARITY && !isStopRequested()) {
                traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .addDisplacementMarker(() -> {
                            ridicareSlide.setPower(0.2);
                            ridicareSlide.setTargetPosition(100);
                        })
                        .lineToLinearHeading(new Pose2d(0, 0, 0))
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

        for (int i = 0; i < v.size(); ++i) {
            pack = new TelemetryPacket();
            pack.put("id", i);
            pack.put("t", v.get(i));
            pack.put("xe", e.get(i).getX());
            pack.put("ye", e.get(i).getY());
            pack.put("he", Math.toDegrees(e.get(i).getHeading()));
            dashboard.sendTelemetryPacket(pack);
        }

    }
}
