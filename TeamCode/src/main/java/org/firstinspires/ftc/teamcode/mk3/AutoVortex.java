package org.firstinspires.ftc.teamcode.mk3;

import static org.firstinspires.ftc.teamcode.RobotVars.DOT;
import static org.firstinspires.ftc.teamcode.RobotVars.EMAX;
import static org.firstinspires.ftc.teamcode.RobotVars.EMIN;
import static org.firstinspires.ftc.teamcode.RobotVars.EXTT;
import static org.firstinspires.ftc.teamcode.RobotVars.RBOT_POS;
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
import static org.firstinspires.ftc.teamcode.RobotVars.SHP;
import static org.firstinspires.ftc.teamcode.RobotVars.SINCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.UPT;
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
import static org.firstinspires.ftc.teamcode.RobotVars.armHolding;
import static org.firstinspires.ftc.teamcode.RobotVars.coneClaw;
import static org.firstinspires.ftc.teamcode.RobotVars.coneReady;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
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

    public int F = 65;

    public static double SPOSX = 0;
    public static double SPOSY = 0;
    public static double SPOSH = 0;

    public static int HMAX = 420;

    public static double HEAD1 = 4.310;
    public static double PX1 = -151;
    public static double PY1 = 42;

    public static boolean AAAAAAAAAAAAAA = false;
    public static boolean BBBBBBBBBBBBBB = true;
    public static boolean RECURRING_SINGULARITY = false;
    public static boolean GPOS = false;

    public static double MVEL = 170;
    public static double MAL = 130;
    public static double MDL = 100;

    public static double WD = 0.2;
    public static double AAA = 0;
    public static double WD2 = 1.3;

    public static double R1X = 30;
    public static double R1Y = 4;
    public static double R2X = 30;
    public static double R2Y = -2;

    public static double RD = -0.7;
    public static double RD2 = 0.0;
    public static double TPT = 0.4;
    public static double GW = 0.9;
    public static double DW = 0.4;
    public static double ET = 0.4;

    int lp = 1;
    public static double ST = 0.452;
    public static double SD = -0.004;
    public static double SBT = 0.75;
    public static double SBD = 0.00;
    public static double SHT = 0.0;
    public static double SHD = 0.00;

    public static double PAX = -130.00;
    public static double PAY6 = -130.00;
    public static double PAY7 = 19.00;
    public static double PAY8 = 82.00;
    public static double PAH = 5.51;

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
            pack.put("Trajex", traj.end().getX());
            pack.put("Trajey", traj.end().getY());
            pack.put("Trajeh", traj.end().getHeading());
            pack.put("Trajsx", traj.start().getX());
            pack.put("Trajsy", traj.start().getY());
            pack.put("Trajsh", traj.start().getHeading());
            pack.put("CycleTime", timer.milliseconds());
            timer.reset();
            dashboard.sendTelemetryPacket(pack);
        }
    }


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
        sHeading.setPosition(SHP);
        clo.toPut = true;
    }

    void set_grab_pos(int p) {
        conversiePerverssa(ST - SD * (p - 1));
        sBalans.setPosition(SBT - SBD * (p - 1));
        sHeading.setPosition(SHT - SHD * (p - 1));
    }

    void upd_grab_pos() {
        set_grab_pos(lp);
        ++lp;
    }

    public static int NUMC = 0;

    TrajectorySequence mktraj() {
        Vector2d R1 = new Vector2d(R1X, R1Y);
        Vector2d R2 = new Vector2d(R2X, R2Y);
        TrajectoryVelocityConstraint vc = SampleMecanumDrive.getVelocityConstraint(MVEL, MAX_ANG_VEL, TRACK_WIDTH);
        TrajectoryAccelerationConstraint ac = SampleMecanumDrive.getAccelerationConstraint(MAL);
        TrajectoryAccelerationConstraint dc = SampleMecanumDrive.getAccelerationConstraint(MDL);
        lp = 1;
        TrajectorySequenceBuilder t = drive.trajectorySequenceBuilder(new Pose2d(SPOSX, SPOSY, SPOSH))
                .funnyRaikuCurveLinear(new Pose2d(PX1, PY1, HEAD1), R1, R2, vc, ac, dc)
                .UNSTABLE_addTemporalMarkerOffset(RD, () -> {
                    rid(RTOP_POS);
                    set_grab_pos(lp + 1);
                })
                .waitSeconds(WD)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    rid(RBOT_POS);
                    set_grab_pos(lp + 1);
                });
        for (int i = 0; i < NUMC; ++i) {
            t.funnyRaikuCurveLinear(new Pose2d(PX1 + 0.00001 * (i + 1), PY1, HEAD1), new Vector2d(0.00001, 0), new Vector2d(0.000001, 0))
                    .waitSeconds(AAA)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        getpos();
                        epd.set_target(HMAX, EXTT);
                        upd_grab_pos();
                        getpos();
                    })
                    .waitSeconds(ET)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        sClose.setPosition(SINCHIS);
                    })
                    .waitSeconds(0.11)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        conversiePerverssa(SAH);
                    })
                    .waitSeconds(TPT)
                    .UNSTABLE_addTemporalMarkerOffset(0, this::ret)/// GET CONE
                    .waitSeconds(GW)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        clo.toGet = true;
                        sClose.setPosition(SDESCHIS);
                        sMCLaw.setPosition(SCC);
                        conversiePerverssa(SAH);
                    })
                    .waitSeconds(DW)
                    .UNSTABLE_addTemporalMarkerOffset(RD2, () -> {
                        rpd.set_target(RTOP_POS, UPT);
                        set_grab_pos(lp + 1);
                    })
                    .waitSeconds(WD2)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                        getpos();
                        rid(RBOT_POS);
                        set_grab_pos(lp + 1);
                    }); /// PUT CONE
        }
        return t.waitSeconds(0.2)
                .build();
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
                    conversiePerverssa(SAW);
                    sMCLaw.setPosition(SCO);
                }
                TA = gamepad1.b;

                if (gamepad1.y && !TB) {
                    epd.set_target(HMAX, EXTT);
                }
                TB = gamepad1.y;

                if (gamepad1.x && !TX) {
                    set_grab_pos(PUST);
                }
                TX = gamepad1.x;
            }
        }
    }

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


        TrajectorySequence traj = null;

        if (BBBBBBBBBBBBBB) {
            traj = mktraj();
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

        {
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

                it = getRuntime();
                follow_traj(traj);

                if (!isStopRequested()) {
                    switch (LAST_ID) {
                        default:
                            telemetry.addLine("DEFAULT");
                        case 7:
                            telemetry.addLine("7");
                            traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    //.funnyRaikuCurve(new Pose2d(F * 2.15, 0, 0), new Vector2d(20, Math.PI), new Vector2d(0.00001, 0.0), vc, ac)
                                    .addTemporalMarker(() -> {
                                        conversiePerverssa(SAW);
                                        sClose.setPosition(SDESCHIS);
                                        sHeading.setPosition(SHG);
                                        sBalans.setPosition(SBAG);
                                        ext(EMIN);
                                        getpos();
                                    })
                                    .lineToLinearHeading(new Pose2d(PAX, PAY7, PAH))
                                    .addTemporalMarker(this::ltime)
                                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> ext(EMIN))
                                    .waitSeconds(1)
                                    .build();
                            break;
                        case 6:
                            telemetry.addLine("6");
                            traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    //.funnyRaikuCurve(new Pose2d(F * 2.15, F * 1, 0), new Vector2d(20, Math.PI), new Vector2d(0.00001, 0.0), vc, ac)
                                    .addTemporalMarker(() -> {
                                        conversiePerverssa(SAW);
                                        sClose.setPosition(SDESCHIS);
                                        sHeading.setPosition(SHG);
                                        sBalans.setPosition(SBAG);
                                        ext(EMIN);
                                        getpos();
                                    })
                                    .lineToLinearHeading(new Pose2d(PAX, PAY7, PAH))
                                    .addTemporalMarker(this::getpos)
                                    .waitSeconds(0.1)
                                    .lineToLinearHeading(new Pose2d(PAX, PAY6, PAH))
                                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> ext(EMIN))
                                    .addTemporalMarker(this::ltime)
                                    .waitSeconds(1)
                                    .build();
                            break;
                        case 8:
                            telemetry.addLine("8");
                            traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    //.funnyRaikuCurve(new Pose2d(F * 2.15, -F * 1, 0), new Vector2d(20, Math.PI), new Vector2d(0.00001, 0.0), vc, ac)
                                    .addTemporalMarker(() -> {
                                        conversiePerverssa(SAW);
                                        sClose.setPosition(SDESCHIS);
                                        sHeading.setPosition(SHG);
                                        sBalans.setPosition(SBAG);
                                        ext(EMIN);
                                        getpos();
                                    })
                                    .lineToLinearHeading(new Pose2d(PAX, PAY7, PAH))
                                    .addTemporalMarker(this::getpos)
                                    .waitSeconds(0.1)
                                    .lineToLinearHeading(new Pose2d(PAX, PAY8, PAH))
                                    .addTemporalMarker(this::ltime)
                                    .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> ext(EMIN))
                                    .waitSeconds(1)
                                    .build();
                            break;
                    }
                    telemetry.update();
                    follow_traj(traj);
                }
            }

            if (GPOS) {
                getpos();
            }

            if (RECURRING_SINGULARITY && !isStopRequested()) {
                traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .addDisplacementMarker(() -> ext(EMIN))
                        .lineToLinearHeading(new Pose2d(0, 0, 0))
                        .build();
                follow_traj(traj);
            }
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
