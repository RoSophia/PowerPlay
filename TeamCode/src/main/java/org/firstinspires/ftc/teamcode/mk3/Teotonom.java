package org.firstinspires.ftc.teamcode.mk3;

import static org.firstinspires.ftc.teamcode.RobotVars.DOT;
import static org.firstinspires.ftc.teamcode.RobotVars.EMIN;
import static org.firstinspires.ftc.teamcode.RobotVars.RBOT_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.RETT;
import static org.firstinspires.ftc.teamcode.RobotVars.RTOP_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.SAH;
import static org.firstinspires.ftc.teamcode.RobotVars.SAP;
import static org.firstinspires.ftc.teamcode.RobotVars.SAW;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAG;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAH;
import static org.firstinspires.ftc.teamcode.RobotVars.SCC;
import static org.firstinspires.ftc.teamcode.RobotVars.SCO;
import static org.firstinspires.ftc.teamcode.RobotVars.SDESCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.SHG;
import static org.firstinspires.ftc.teamcode.RobotVars.ep;
import static org.firstinspires.ftc.teamcode.RobotVars.SHITTY_WORKAROUND_POWER;
import static org.firstinspires.ftc.teamcode.RobotVars.SHITTY_WORKAROUND_TIME;
import static org.firstinspires.ftc.teamcode.RobotVars.SINCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.armHolding;
import static org.firstinspires.ftc.teamcode.RobotVars.coneClaw;
import static org.firstinspires.ftc.teamcode.RobotVars.coneReady;
import static org.firstinspires.ftc.teamcode.RobotVars.pcoef;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.clo;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.SHOULD_CLOSE_IMU;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.conversiePerverssa;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.dashboard;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.endma;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.epd;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.ext;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.extA;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.extB;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.initma;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.leftBack;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.leftEncoder;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.leftFront;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.log_state;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.preinit;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.rid;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.rightBack;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.rightFront;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.rpd;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sBalans;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sClose;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sHeading;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sMCLaw;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.set_grab_pos;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.startma;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.wtfor;
import static org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner.COLOR_INACTIVE_TRAJECTORY;
import static org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner.COLOR_INACTIVE_TURN;
import static org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner.COLOR_INACTIVE_WAIT;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mk3.camera.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.mk3.camera.CamGirl;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.time.chrono.ThaiBuddhistEra;
import java.util.ArrayList;
import java.util.Vector;

@Config
@Autonomous(group = "drive", name = "Auto Mediu S")
@SuppressLint("DefaultLocale")
public class Teotonom extends LinearOpMode {
    AprilTagDetectionPipeline qtPipeline;
    SampleMecanumDrive drive;

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

    public static double P1H = 5.585;
    public static double P1X = -131;
    public static double P1Y = 14;
    public static double P2H = 4.59;
    public static double P2X = -125;
    public static double P2Y = -3;
    public static double P3H = 4.492;
    public static double P3X = -95.5;
    public static double P3Y = 6.7;

    public static double P678X = -85;
    public static double P678H = -0;
    public static double P6Y = -65;
    public static double P7Y = 4;
    public static double P8Y = 65;

    public static double P71X = 40;
    public static double P71Y = 3.14;
    public static double P72X = 30;
    public static double P72Y = 2.7;

    public static double P81X = 50;
    public static double P81Y = 3.14;
    public static double P82X = 65;
    public static double P82Y = 2.5;

    public static double P61X = 60;
    public static double P61Y = 2.9;
    public static double P62X = 60;
    public static double P62Y = 3.1;

    public static double RAI1X = 20;
    public static double RAI1Y = -4;
    public static double RAI2X = 30;
    public static double RAI2Y = -3.2;

    public static boolean BBBBBBBBBBBBBB = true;
    public static boolean RECURRING_SINGULARITY = false;
    public static boolean GPOS = false;

    public static double MVEL = 170;
    public static double MAL = 140;
    public static double MDL = 80;
    /*
    public static double MVEL = 999; // THE ONLY TYPE OF
    public static double MAL = 999;  // BREAKING I KNOW IS
    public static double MDL = 999;  // BREAKING BAD
     */

    public static double R1X = 26;
    public static double R1Y = -2.4;
    public static double R2X = 26;
    public static double R2Y = -1.2;

    public static int EM = 470;
    public static int EMS = 470;
    public static int EME = 450;

    Vector<Double> v = new Vector<>();
    Vector<Pose2d> e = new Vector<>();
    double it;

    void ltime() {
        v.add(getRuntime() - it);
        e.add(drive.getLastError());
    }

    double CUR_CORRECTION = 0.0;

    void set_wait_time(double t) {
        drive.follower = new HolonomicPIDVAFollower(SampleMecanumDrive.AXIAL_PID, SampleMecanumDrive.LATERAL_PID, SampleMecanumDrive.HEADING_PID, new Pose2d(2, 2, Math.toRadians(2)), t);
        drive.trajectorySequenceRunner = new TrajectorySequenceRunner(drive.follower, SampleMecanumDrive.HEADING_PID);
        CUR_CORRECTION = t;
    }

    void follow_traj(TrajectorySequence traj) {
        if (traj == null || isStopRequested()) {
            return;
        }
        telemetry.addLine("START FOL!");
        telemetry.update();
        drive.followTrajectorySequenceAsync(traj);
        telemetry.addLine("START FOL 2!");
        telemetry.update();
        ElapsedTime timer = new ElapsedTime(0);
        ElapsedTime FULL_TIMER = new ElapsedTime(0);
        FULL_TIMER.reset();
        double MAX_DURATION = traj.duration();
        TelemetryPacket telepack = new TelemetryPacket();
        telepack.put("TEMP:MAX_DUR", MAX_DURATION);
        telepack.put("TEMP:FULL_TIMER", FULL_TIMER.seconds());
        dashboard.sendTelemetryPacket(telepack);
        telemetry.addLine("STARTED TO DRIVE!");
        telemetry.update();
        while (drive.isBusy() && !isStopRequested() && !gamepad1.right_bumper && FULL_TIMER.seconds() <= MAX_DURATION + CUR_CORRECTION + 0.01) {
            drive.update();
            telemetry.update();
            leftEncoder.getCorrectedVelocity();
            log_state();
            TelemetryPacket pack = new TelemetryPacket();
            pack.put("CycleTime", timer.milliseconds());
            pack.put("TargH", traj.end().getHeading());
            pack.put("TargX", traj.end().getX());
            pack.put("TargY", traj.end().getY());
            timer.reset();
            dashboard.sendTelemetryPacket(pack);

            telepack = new TelemetryPacket();
            telepack.put("TEMP:MAX_DUR", MAX_DURATION);
            telepack.put("TEMP:FULL_TIMER", FULL_TIMER.seconds());
            dashboard.sendTelemetryPacket(telepack);

            if (!SHITTY_WORKAROUND_TIMED && SHITTY_WORKAROUND_TIMER.seconds() < SHITTY_WORKAROUND_TIME) {
                if (extA != null) {
                    epd.use = false;
                    epd.curRet = true;
                    extA.setPower(-SHITTY_WORKAROUND_POWER);
                    extB.setPower(-SHITTY_WORKAROUND_POWER);
                }
            } else if (!SHITTY_WORKAROUND_TIMED) {
                SHITTY_WORKAROUND_TIMED = true;
                if (extA != null) {
                    extA.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    extA.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    extB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    extB.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                }
                epd.use = true;
                epd.curRet = false;
            }
        }
        telemetry.addLine("FINISHED TO DRIVE!");
        telemetry.update();
        drive.update();
        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
        drive.tl.updHeading();
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

    public static double RD = -1.0;

    public static double WHO = 0.15;
    public static double WEX = 0.05;

    int lp = 1;

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
        clo.toPut = false;
    }

    public void st_grab_pos() {
        set_grab_pos(lp);
    }

    void upd_grab_pos() {
        openClaw();
        set_grab_pos(lp);
        ++lp;
    }

    public static int NUMC = 5;

    TrajectorySequence goToPreload;
    TrajectorySequence preloadToGet;
    TrajectorySequence keepPos;
    TrajectorySequence goToPark;

    Tecton ihk;
    Thread ihkT;

    void openClaw() {
        clo.clw.reset();
        clo._clw = false;
    }

    double MV = 60;
    double MA = 40;
    void mktraj() {
        Vector2d R1 = new Vector2d(R1X, R1Y);
        Vector2d R2 = new Vector2d(R2X, R2Y);
        TrajectoryVelocityConstraint vvc = SampleMecanumDrive.getVelocityConstraint(MV, MAX_ANG_VEL, TRACK_WIDTH);
        TrajectoryAccelerationConstraint aac = SampleMecanumDrive.getAccelerationConstraint(MA);

        TrajectoryVelocityConstraint vc = SampleMecanumDrive.getVelocityConstraint(MVEL, MAX_ANG_VEL, TRACK_WIDTH);
        TrajectoryAccelerationConstraint ac = SampleMecanumDrive.getAccelerationConstraint(MAL);
        TrajectoryAccelerationConstraint dc = SampleMecanumDrive.getAccelerationConstraint(MDL);
        lp = 1;

        goToPreload = drive.trajectorySequenceBuilder(new Pose2d(SPOSX, SPOSY, SPOSH))
                .funnyRaikuCurveLinear(new Pose2d(P1X, P1Y, P1H), R1, R2, vc, ac, dc)
                .UNSTABLE_addTemporalMarkerOffset(RD, () -> rid(RTOP_POS))
                .build();

        preloadToGet = drive.trajectorySequenceBuilder(new Pose2d(P1X + 0.00001, P1Y, P1H))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    rid(RBOT_POS);
                    ret();
                    set_grab_pos(1);
                })
                .lineToLinearHeading(new Pose2d(P2X, P2Y, P2H), vvc, aac)
                .lineToLinearHeading(new Pose2d(P3X, P3Y, P3H), vvc, aac)
                .build();

        keepPos = drive.trajectorySequenceBuilder(new Pose2d(P3X + 0.00001, P3Y, P3H))
                .lineToLinearHeading(new Pose2d(P3X, P3Y, P3H), vvc, aac)
                .build();

        TelemetryPacket tatp = new TelemetryPacket();
        tatp.put("CURRENT LAST ID KMS", LAST_ID);
        dashboard.sendTelemetryPacket(tatp);
    }

    void make_park() {
        TrajectoryVelocityConstraint vvc = SampleMecanumDrive.getVelocityConstraint(MV, MAX_ANG_VEL, TRACK_WIDTH);
        TrajectoryAccelerationConstraint aac = SampleMecanumDrive.getAccelerationConstraint(MA);
        TrajectoryAccelerationConstraint ddc = SampleMecanumDrive.getAccelerationConstraint(MA);
        switch (LAST_ID) {
            default:
            case 7:
                goToPark = drive.trajectorySequenceBuilder(new Pose2d(P3X, P3Y - 10, P3H))
                        .addTemporalMarker(() -> {
                            conversiePerverssa(SAW);
                            sClose.setPosition(SINCHIS);
                            sHeading.setPosition(SHG);
                            sBalans.setPosition(SBAG);
                            ext(EMIN);
                        })
                        .funnyRaikuCurveLinear(new Pose2d(P678X, P7Y, P678H), new Vector2d(P71X, P71Y), new Vector2d(P72X, P72Y), vvc, aac, ddc)
                        .addTemporalMarker(this::ltime)
                        .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> ext(EMIN))
                        .waitSeconds(1)
                        .build();
                break;
            case 6:
                goToPark = drive.trajectorySequenceBuilder(new Pose2d(P3X, P3Y - 10, P3H))
                        .addTemporalMarker(() -> {
                            conversiePerverssa(SAW);
                            sClose.setPosition(SINCHIS);
                            sHeading.setPosition(SHG);
                            sBalans.setPosition(SBAG);
                            ext(EMIN);
                        })
                        .funnyRaikuCurveLinear(new Pose2d(P678X, P6Y, P678H), new Vector2d(P61X, P61Y), new Vector2d(P62X, P62Y), vvc, aac, ddc)
                        .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> ext(EMIN))
                        .addTemporalMarker(this::ltime)
                        .waitSeconds(1)
                        .build();
                break;
            case 8:
                goToPark = drive.trajectorySequenceBuilder(new Pose2d(P3X, P3Y, P3H))
                        .addTemporalMarker(() -> {
                            conversiePerverssa(SAW);
                            sClose.setPosition(SINCHIS);
                            sHeading.setPosition(SHG);
                            sBalans.setPosition(SBAG);
                            ext(EMIN);
                        })
                        .funnyRaikuCurveLinear(new Pose2d(P678X, P8Y, P678H), new Vector2d(P81X, P81Y), new Vector2d(P82X, P82Y), vvc, aac, ddc)
                        .addTemporalMarker(this::ltime)
                        .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> ext(EMIN))
                        .waitSeconds(1)
                        .build();
                break;
        }
    }

    boolean TA = false;
    boolean TB = false;
    boolean TX = false;
    public static int PUST = 1;

    void getpos() {
        TA = TB = TX = false;
        boolean SGP = false;
        boolean DL = false;
        boolean DR = false;
        boolean DU = false;
        if (GPOS) {
            while (!isStopRequested() && !gamepad1.right_bumper) {
                telemetry.addData("PE", drive.tl.getPoseEstimate());
                telemetry.addData("PEH", drive.tl.getPoseEstimate().getHeading() / 180 * Math.PI);
                telemetry.addData("Pe", drive.tl.getLastError());
                telemetry.update();
                log_state();
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
                    sClose.setPosition(SDESCHIS);
                    epd.set_target(EM, 0);
                }
                TB = gamepad1.y;

                if (gamepad1.x && !TX) {
                    SGP = !SGP;
                }
                TX = gamepad1.x;
                if (SGP) {
                    set_grab_pos(PUST);
                }

                if (gamepad1.dpad_left && !DL) {
                    conversiePerverssa(SAH);
                    sBalans.setPosition(SBAH);
                }
                DL = gamepad1.dpad_left;

                if (gamepad1.dpad_right && !DR) {
                    if (sClose.getPosition() != SINCHIS) {
                        sClose.setPosition(SINCHIS);
                    } else {
                        sClose.setPosition(SDESCHIS);
                    }
                }
                DR = gamepad1.dpad_right;

                if (gamepad1.dpad_up && !DU) {
                    clo.toPut = true;
                }
                DU = gamepad1.dpad_up;

                if (!SHITTY_WORKAROUND_TIMED && SHITTY_WORKAROUND_TIMER.seconds() < SHITTY_WORKAROUND_TIME) {
                    if (extA != null) {
                        epd.use = false;
                        epd.curRet = true;
                        extA.setPower(-SHITTY_WORKAROUND_POWER);
                        extB.setPower(-SHITTY_WORKAROUND_POWER);
                    }
                } else if (!SHITTY_WORKAROUND_TIMED) {
                    SHITTY_WORKAROUND_TIMED = true;
                    if (extA != null) {
                        extA.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        extA.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        extB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        extB.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    }
                    epd.use = true;
                    epd.curRet = false;
                }
            }
        }
    }

    ElapsedTime SHITTY_WORKAROUND_TIMER = new ElapsedTime(0);
    boolean SHITTY_WORKAROUND_TIMED = false;

    public static double WOT = 0.02;
    public static double WAT = 0.15;

    void runBBBBBBBBBBBBBB() {
        clo.shouldClose = true;
        rpd.shouldClose = true;
        epd.shouldClose = true;
        telemetry.addLine("Start");
        telemetry.update();
        double P11X = 0, P11Y = 0, P12X = 0, P12Y = 0;

        while (!isStopRequested()) {
            if (P11X != RAI1X || P12X != RAI2X || P11Y != RAI1Y || P12Y != RAI2Y) {
                mktraj();
                make_park();
                if (goToPreload == null) {
                    continue;
                }
                TelemetryPacket p = new TelemetryPacket();
                Canvas fieldOverlay = p.fieldOverlay();
                draw(fieldOverlay, goToPreload);
                draw(fieldOverlay, preloadToGet);
                p.put("Updated!", 0);
                dashboard.sendTelemetryPacket(p);
                P11X = RAI1X;
                P11Y = RAI1Y;
                P12X = RAI2X;
                P12Y = RAI2Y;
            }

            sleep(100);
            telemetry.addLine("THIS IS BBBBBBBBBBBBBBBBBBBBBB");
            telemetry.addLine("____  ____  ____  ____  ____  ____  ____");
            telemetry.addLine("| __ )| __ )| __ )| __ )| __ )| __ )| __ ) ");
            telemetry.addLine("|  _ \\|  _ \\|  _ \\|  _ \\|  _ \\|  _ \\|  _ \\ ");
            telemetry.addLine("| |_) | |_) | |_) | |_) | |_) | |_) | |_) |");
            telemetry.addLine("|____/|____/|____/|____/|____/|____/|____/");
            telemetry.update();

        }
    }


    CamGirl qtGirl;

    void init_auto() {
        initma(hardwareMap);
        sMCLaw.setPosition(SCC);
        ihk = new Tecton(this);
        ihkT = new Thread(ihk);
        drive = new SampleMecanumDrive(hardwareMap);
        RobotFuncs.drive = drive;
        coneReady = true;

        qtPipeline = new AprilTagDetectionPipeline(TAGSIZE, FX, FY, CX, CY);
        qtGirl = new CamGirl(this, "qtGirl", OpenCvCameraRotation.SIDEWAYS_LEFT, 640, 480, qtPipeline, true, false);

        if (BBBBBBBBBBBBBB) {
            clo.shouldClose = true;
            rpd.shouldClose = true;
            epd.shouldClose = true;
        }

        if (!isStopRequested()) {
            mktraj();
        }

        TelemetryPacket packet;
        while (!isStarted() && !isStopRequested()) {
            if (qtGirl.getOpened()) {
                if (LAST_ID == 0) {
                    telemetry.addLine("Cam opened");
                    telemetry.update();
                }
                ArrayList<AprilTagDetection> cd = qtPipeline.getLatestDetections();
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

        if (!isStopRequested()) {
            make_park();
        }

        telemetry.addData("All done! Got ID: ", LAST_ID);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        ep = 0.0023;
        preinit();
        init_auto();

        if (isStopRequested()) {
            return;
        }

        TelemetryPacket tttp = new TelemetryPacket();
        tttp.addLine("INIT DONE!");
        telemetry.addLine("INIT DONE!");
        telemetry.update();
        waitForStart();
        if (qtGirl.getOpened()) {
            qtGirl.stop();
        }

        startma(this, telemetry);
        telemetry.addLine("START DONE!");
        telemetry.update();
        sMCLaw.setPosition(SCC);
        ihk.shouldClose = false;
        ihk.lom = this;
        ihkT.start();
        sClose.setPosition(SINCHIS);
        telemetry.addLine("ASTART DONE!");
        telemetry.update();

        if (!BBBBBBBBBBBBBB) {
            runBBBBBBBBBBBBBB();
        } else {
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("LID", LAST_ID);
            dashboard.sendTelemetryPacket(packet);

            it = getRuntime();

            SHITTY_WORKAROUND_TIMER.reset();
            SHITTY_WORKAROUND_TIMED = false;
            telemetry.addLine("BEFOR GETPOS!");
            telemetry.update();
            getpos();
            telemetry.addLine("AFTR GETPOS!");
            telemetry.update();
            set_wait_time(WOT);
            follow_traj(goToPreload);
            telemetry.addLine("AFTR FOLLOW!");
            telemetry.update();
            getpos();
            wtfor(RobotFuncs.WAITS.HOISTERR, WHO); // WAIT FOR BETTER NO EXTRA WAIT IF WAITING IN DRUM
            set_wait_time(WAT);
            follow_traj(preloadToGet);
            EM = EMS;
            epd.set_target(EM, 0);
            upd_grab_pos();
            wtfor(RobotFuncs.WAITS.EXTENSIONE, WEX);
            getpos();
            ihk.stage = 1;
            wtfor(RobotFuncs.WAITS.TRANSFER, WEX);
            getpos();
            set_wait_time(WAT);
            for (int i = 0; i < NUMC - 1; ++i) {
                wtfor(RobotFuncs.WAITS.HOISTERR, WHO);
                rid(RBOT_POS);
                ret();
                follow_traj(keepPos);
                getpos();
                leftFront.setPower(0.001);
                leftBack.setPower(0.001);
                rightFront.setPower(0.001);
                rightBack.setPower(0.001);
                if (i > 2) {
                    EM = EME;
                }
                epd.set_target(EM, 0);
                upd_grab_pos();
                wtfor(RobotFuncs.WAITS.EXTENSIONE, WEX);
                getpos();
                ihk.stage = 1;
                wtfor(RobotFuncs.WAITS.TRANSFER, WEX);
                leftFront.setPower(0.0);
                leftBack.setPower(0.0);
                rightFront.setPower(0.0);
                rightBack.setPower(0.0);
                getpos();
            }

            if (NUMC >= 1) {
                wtfor(RobotFuncs.WAITS.HOISTERR, WHO);
                rid(RBOT_POS);
                ret();
            }
            getpos();

            set_wait_time(1);
            follow_traj(goToPark);

            if (RECURRING_SINGULARITY && !isStopRequested()) {
                TrajectorySequence traj = drive.trajectorySequenceBuilder(drive.tl.getPoseEstimate())
                        .addDisplacementMarker(() -> ext(EMIN))
                        .lineToLinearHeading(new Pose2d(0, 0, 0))
                        .build();
                follow_traj(traj);
            }

            SHOULD_CLOSE_IMU = false;
            conversiePerverssa(SAP);
            sMCLaw.setPosition(SCC);
            endma();
            ihk.shouldClose = true;

            leftBack.setPower(0);
            rightBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
            ihkT.join();

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
