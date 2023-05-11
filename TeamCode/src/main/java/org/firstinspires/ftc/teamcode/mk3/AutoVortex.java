package org.firstinspires.ftc.teamcode.mk3;

import static org.firstinspires.ftc.teamcode.RobotVars.DOT;
import static org.firstinspires.ftc.teamcode.RobotVars.EMAX;
import static org.firstinspires.ftc.teamcode.RobotVars.EMIN;
import static org.firstinspires.ftc.teamcode.RobotVars.EXTT;
import static org.firstinspires.ftc.teamcode.RobotVars.FER;
import static org.firstinspires.ftc.teamcode.RobotVars.FES;
import static org.firstinspires.ftc.teamcode.RobotVars.LER;
import static org.firstinspires.ftc.teamcode.RobotVars.LES;
import static org.firstinspires.ftc.teamcode.RobotVars.RBOT_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.RER;
import static org.firstinspires.ftc.teamcode.RobotVars.RES;
import static org.firstinspires.ftc.teamcode.RobotVars.RETT;
import static org.firstinspires.ftc.teamcode.RobotVars.RTOP_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.SAH;
import static org.firstinspires.ftc.teamcode.RobotVars.SAP;
import static org.firstinspires.ftc.teamcode.RobotVars.SAW;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAG;
import static org.firstinspires.ftc.teamcode.RobotVars.SCC;
import static org.firstinspires.ftc.teamcode.RobotVars.SCO;
import static org.firstinspires.ftc.teamcode.RobotVars.SDESCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.SHG;
import static org.firstinspires.ftc.teamcode.RobotVars.SINCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.armHolding;
import static org.firstinspires.ftc.teamcode.RobotVars.coneClaw;
import static org.firstinspires.ftc.teamcode.RobotVars.coneReady;
import static org.firstinspires.ftc.teamcode.RobotVars.pcoef;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.batteryVoltageSensor;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.clo;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.conversiePerverssa;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.dashboard;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.endma;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.epd;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.ext;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.imu;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.initma;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.leftBack;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.leftFront;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.rid;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.rightBack;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.rightFront;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.rpd;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sBalans;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sClose;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sHeading;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sMCLaw;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.startma;
import static org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner.COLOR_INACTIVE_TRAJECTORY;
import static org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner.COLOR_INACTIVE_TURN;
import static org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner.COLOR_INACTIVE_WAIT;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
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

@Config
@Autonomous(group = "drive")
@SuppressLint("DefaultLocale")
public class AutoVortex extends LinearOpMode {
    OpenCvCamera webcam;
    AprilTagDetectionPipeline pipeline;
    SampleMecanumDrive drive;

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

    @SuppressWarnings("unused")
    ThaiBuddhistEra thaiBuddhistEra; // 777hz tibetan healing sounds

    public static double SPOSX = 0;
    public static double SPOSY = 0;
    public static double SPOSH = 0;

    // -38.7 -57.7 117.2
    // 206.6 8.318 28.337

    public static double HEAD1 = 0.0;
    public static double PX1 = -204;
    public static double PY1 = -36;
    public static double HEAD2 = 0.0;
    public static double PX2 = -196;
    public static double PY2 = -36;

    public static double PAX = -193;
    public static double PAH = 0;
    public static double PAY6 = -40;
    public static double PAY7 = 0;
    public static double PAY8 = 40;


    public static double P1X = 30;
    public static double P1Y = 0.7;
    public static double P2X = 30;
    public static double P2Y = 0;

    public static boolean BBBBBBBBBBBBBB = true;
    public static boolean RECURRING_SINGULARITY = true;
    public static boolean GPOS = true;

    public static double MVEL = 170;
    public static double MAL = 130;
    public static double MDL = 100;

    public double WD = 0.02;
    public static double WD2 = 0.9;

    public static double R1X = 30;
    public static double R1Y = 3.0;
    public static double R2X = 30;
    public static double R2Y = 1.5;

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
        TelemetryPacket pack;
        ElapsedTime timer = new ElapsedTime(0);
        while (drive.isBusy() && !isStopRequested() && traj != null && !gamepad1.right_bumper) {
            drive.update();
            pack = new TelemetryPacket();
            pack.put("Ex", drive.getLastError().getX());
            pack.put("Ey", drive.getLastError().getY());
            pack.put("Eh", drive.getLastError().getHeading());
            pack.put("CycleTime", timer.milliseconds());
            pack.put("vel", leftEncoder.getCorrectedVelocity());
            pack.put("ver", rightEncoder.getCorrectedVelocity());
            pack.put("vef", frontEncoder.getCorrectedVelocity());
            timer.reset();
            dashboard.sendTelemetryPacket(pack);
        }
    }

    final double ITC = 1 / 2.54;

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

    public static double RD = -0.7;
    public static double RD2 = -0.4;
    public static double GW = 0.5;
    public static double ET = 0.6;

    int lp = 1;
    public static double ST = 0.452;
    public static double SD = -0.004;

    public static int EX = 300;
    public static double EXT = 1.0;

    void ret() {
        armHolding = false;
        coneClaw = false;
        clo.cget = false;
        clo.cprepCone = false;
        clo.cput = false;
        coneReady = false;
        clo.toGet = false;
        clo.tppc = false;
        clo.toPrepCone = false;
        clo.timt = 1;
        sClose.setPosition(SDESCHIS);
        clo.toPut = true;
    }

    void set_grab_pos(int p) {
        conversiePerverssa(ST - SD * (p - 1));
    }

    void upd_grab_pos() {
        set_grab_pos(lp);
        ++lp;
    }

    public static int NUMC = 4;

    TrajectorySequence goToStalp;
    TrajectorySequence extend;
    TrajectorySequence intend;
    TrajectorySequence goToPark;

    void mktraj() {
        Vector2d P1 = new Vector2d(P1X, P1Y);
        Vector2d P2 = new Vector2d(P2X, P2Y);
        Vector2d R1 = new Vector2d(R1X, R1Y);
        Vector2d R2 = new Vector2d(R2X, R2Y);
        TrajectoryVelocityConstraint vc = SampleMecanumDrive.getVelocityConstraint(MVEL, MAX_ANG_VEL, TRACK_WIDTH);
        TrajectoryAccelerationConstraint ac = SampleMecanumDrive.getAccelerationConstraint(MAL);
        TrajectoryAccelerationConstraint dc = SampleMecanumDrive.getAccelerationConstraint(MDL);
        lp = 1;

        goToStalp = drive.trajectorySequenceBuilder(new Pose2d(SPOSX, SPOSY, SPOSH))
                .UNSTABLE_addTemporalMarkerOffset(0.0, this::getpos)
                .waitSeconds(0.1)
                .funnyRaikuCurveLinear(new Pose2d(PX1, PY1, HEAD1), R1, R2, vc, ac, dc)
                .UNSTABLE_addTemporalMarkerOffset(RD, () -> rid(RTOP_POS))
                .UNSTABLE_addTemporalMarkerOffset(0.0, this::ltime)
                .waitSeconds(WD)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    rid(RBOT_POS);
                    ret();
                })
                .funnyRaikuCurveLinear(new Pose2d(PX2, PY2, HEAD2), P1, P2)
                .build();

        extend = drive.trajectorySequenceBuilder(new Pose2d(PX2 + 0.00001, PY2, HEAD2))
                .lineToLinearHeading(new Pose2d(PX2, PY2, HEAD2))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    epd.set_target(EX, EXT);
                    upd_grab_pos();
                })
                .waitSeconds(ET)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> sClose.setPosition(SINCHIS))
                .waitSeconds(0.11)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> conversiePerverssa(SAH))
                .build();

        intend = drive.trajectorySequenceBuilder(new Pose2d(PX2, PY2, HEAD2))
                .waitSeconds(GW)
                .UNSTABLE_addTemporalMarkerOffset(RD2, () -> rid(RTOP_POS))
                .waitSeconds(WD2)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    rid(RBOT_POS);
                    ret();
                })
                .build();

        switch (LAST_ID) {
            default:
            case 7:
                goToPark = drive.trajectorySequenceBuilder(new Pose2d(PX2, PY2, HEAD2))
                        .addTemporalMarker(() -> {
                            conversiePerverssa(SAW);
                            sClose.setPosition(SDESCHIS);
                            sHeading.setPosition(SHG);
                            sBalans.setPosition(SBAG);
                            ext(EMIN);
                        })
                        .lineToLinearHeading(new Pose2d(PAX, PAY7, PAH))
                        .addTemporalMarker(this::ltime)
                        .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> ext(EMIN))
                        .waitSeconds(1)
                        .build();
                break;
            case 6:
                goToPark = drive.trajectorySequenceBuilder(new Pose2d(PX2, PY2, HEAD2))
                        .addTemporalMarker(() -> {
                            conversiePerverssa(SAW);
                            sClose.setPosition(SDESCHIS);
                            sHeading.setPosition(SHG);
                            sBalans.setPosition(SBAG);
                            ext(EMIN);
                        })
                        .lineToLinearHeading(new Pose2d(PAX, PAY6, PAH))
                        .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> ext(EMIN))
                        .addTemporalMarker(this::ltime)
                        .waitSeconds(1)
                        .build();
                break;
            case 8:
                goToPark = drive.trajectorySequenceBuilder(new Pose2d(PX2, PY2, HEAD2))
                        .addTemporalMarker(() -> {
                            conversiePerverssa(SAW);
                            sClose.setPosition(SDESCHIS);
                            sHeading.setPosition(SHG);
                            sBalans.setPosition(SBAG);
                            ext(EMIN);
                        })
                        .lineToLinearHeading(new Pose2d(PAX, PAY8, PAH))
                        .addTemporalMarker(this::ltime)
                        .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> ext(EMIN))
                        .waitSeconds(1)
                        .build();
                break;
        }
    }

    boolean OPENED = false;

    boolean TA = false;
    boolean TB = false;
    boolean TX = false;
    public static int PUST = 1;

    void getpos() {
        TA = TB = TX = false;
        if (GPOS) {
            while (!isStopRequested() && !gamepad1.right_bumper) {
                drive.updatePoseEstimate();
                telemetry.addData("PE", drive.getPoseEstimate());
                telemetry.addData("PEH", drive.getPoseEstimate().getHeading() / 180 * Math.PI);
                telemetry.addData("Pe", drive.getLastError());
                telemetry.update();
                TelemetryPacket p = new TelemetryPacket();
                p.put("Rx", drive.getPoseEstimate().getX());
                p.put("Ry", drive.getPoseEstimate().getY());
                p.put("Rh", drive.getPoseEstimate().getHeading() * Math.PI / 180);
                p.put("Ex", drive.getLastError().getX());
                p.put("Ey", drive.getLastError().getY());
                p.put("Eh", drive.getLastError().getHeading());
                dashboard.sendTelemetryPacket(p);

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
                final double spcoef = 1 - 0.6 * gamepad1.right_trigger;
                final double fcoef = pcoef * spcoef * 0.7;
                leftFront.setPower(lfPower * fcoef);
                rightFront.setPower(rfPower * fcoef);
                leftBack.setPower(lbPower * fcoef);
                rightBack.setPower(rbPower * fcoef);

                if (gamepad1.a) {
                    break;
                }
                if (gamepad1.b && !TA) {
                    epd.set_target(EMIN, RETT);
                    rpd.set_target(RBOT_POS, DOT);
                    conversiePerverssa(SAH);
                    sMCLaw.setPosition(SCO);
                }
                TA = gamepad1.b;

                if (gamepad1.y && !TB) {
                    epd.set_target(EX, EXT);
                }
                TB = gamepad1.y;

                if (gamepad1.x && !TX) {
                    set_grab_pos(PUST);
                }
                TX = gamepad1.x;
            }
        }
    }

    public static double TARGET_ANGLE = 2.3;
    void align() {
        double sangle = imu.getAngularOrientation().firstAngle;
        double cdif = TARGET_ANGLE - sangle;
        TelemetryPacket tp;
        drive.turnAsync(cdif);
        while (drive.isBusy() && !isStopRequested() && !gamepad1.right_bumper) {
            tp = new TelemetryPacket();
            tp.put("AlignCDIF", cdif);
            tp.put("AlignSangle", sangle);
            tp.put("AlignCangle", imu.getAngularOrientation().firstAngle);
            dashboard.sendTelemetryPacket(tp);
            drive.update();
        }
        Pose2d cp = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(cp.getX(), cp.getY(), HEAD2));
    }

    Encoder leftEncoder, rightEncoder, frontEncoder;

    @Override
    public void runOpMode() throws InterruptedException {
        initma(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        coneReady = true;
        sMCLaw.setPosition(SCC);
        conversiePerverssa(SAP);
        sClose.setPosition(SDESCHIS);
        sBalans.setPosition(SBAG);
        sHeading.setPosition(SHG);

        {
            TelemetryPacket pack = new TelemetryPacket();
            pack.put("Ex", 0);
            pack.put("Ey", 0);
            pack.put("Eh", 0);
            pack.put("CycleTime", 0);
            pack.put("vel", 0);
            pack.put("ver", 0);
            pack.put("vef", 0);
            dashboard.sendTelemetryPacket(pack);
        }

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, LES));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, RES));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, FES));
        leftEncoder.setDirection(LER ? Encoder.Direction.REVERSE : Encoder.Direction.FORWARD);
        rightEncoder.setDirection(RER ? Encoder.Direction.REVERSE : Encoder.Direction.FORWARD);
        frontEncoder.setDirection(FER ? Encoder.Direction.REVERSE : Encoder.Direction.FORWARD);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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

        {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("x", 0);
            packet.put("y", 0);
            packet.put("heading (deg)", 0);

            packet.put("xError", 0);
            packet.put("yError", 0);
            packet.put("headingError (deg)", 0);
            dashboard.sendTelemetryPacket(packet);
        }

        if (BBBBBBBBBBBBBB) {
            clo.shouldClose = true;
            rpd.shouldClose = true;
            epd.shouldClose = true;
        }

        TelemetryPacket packet;

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
            sleep(10);
        }

        mktraj();

        packet = new TelemetryPacket();
        packet.put("bat", batteryVoltageSensor.getVoltage());
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("All done! Got ID: ", LAST_ID);

        waitForStart();
        if (OPENED) {
            webcam.closeCameraDeviceAsync(() -> {
            });
        }
        startma(this, false);

        drive.setPoseEstimate(new Pose2d(SPOSX, SPOSY, SPOSH));
        //drive.setPoseEstimate(new Pose2d(0, 0, 0));

        if (!BBBBBBBBBBBBBB) {
            clo.shouldClose = true;
            rpd.shouldClose = true;
            epd.shouldClose = true;
            telemetry.addLine("Start");
            telemetry.update();
            double P11X = 0, P11Y = 0, P12X = 0, P12Y = 0;
            double R11X = 0, R11Y = 0, R12X = 0, R12Y = 0;

            while (!isStopRequested()) {
                if (P11X != PX1 || P12X != PX2 || P11Y != PY1 || P12Y != PY2 ||
                        R11X != R1X || R12X != R2X || R11Y != R1Y || R12Y != R2Y) {
                    mktraj();
                    if (goToStalp == null) {
                        continue;
                    }
                    TelemetryPacket p = new TelemetryPacket();
                    Canvas fieldOverlay = p.fieldOverlay();
                    draw(fieldOverlay, goToStalp);
                    draw(fieldOverlay, extend);
                    draw(fieldOverlay, intend);
                    p.put("Updated!", 0);
                    dashboard.sendTelemetryPacket(p);
                    P11X = P1X;
                    P11Y = P1Y;
                    P12X = P2X;
                    P12Y = P2Y;
                }

                sleep(100);
            }
        } else {
            packet = new TelemetryPacket();
            packet.put("LID", LAST_ID);
            dashboard.sendTelemetryPacket(packet);

            it = getRuntime();

            getpos();
            follow_traj(goToStalp);
            /*
            for (int i = 0; i < NUMC; ++i) {
                align();
                follow_traj(extend);
                follow_traj(intend);
            }*/
            getpos();
            follow_traj(goToPark);

            if (RECURRING_SINGULARITY && !isStopRequested()) {
                TrajectorySequence traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .addDisplacementMarker(() -> ext(EMIN))
                        .lineToLinearHeading(new Pose2d(0, 0, 0))
                        .build();
                follow_traj(traj);
            }

            endma();

            leftBack.setPower(0);
            rightBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);

            for (int i = 0; i < v.size(); ++i) {
                packet = new TelemetryPacket();
                packet.put("id", i);
                packet.put("t", v.get(i));
                packet.put("xe", e.get(i).getX());
                packet.put("ye", e.get(i).getY());
                packet.put("he", Math.toDegrees(e.get(i).getHeading()));
                dashboard.sendTelemetryPacket(packet);
            }

        }
    }
}
