package org.firstinspires.ftc.teamcode.mk3;

import static org.firstinspires.ftc.teamcode.RobotVars.DOT;
import static org.firstinspires.ftc.teamcode.RobotVars.EMAX;
import static org.firstinspires.ftc.teamcode.RobotVars.EMIN;
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
import static org.firstinspires.ftc.teamcode.RobotVars.SBAH;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAP;
import static org.firstinspires.ftc.teamcode.RobotVars.SCC;
import static org.firstinspires.ftc.teamcode.RobotVars.SCO;
import static org.firstinspires.ftc.teamcode.RobotVars.SDESCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.SHG;
import static org.firstinspires.ftc.teamcode.RobotVars.SHITTY_WORKAROUND_POWER;
import static org.firstinspires.ftc.teamcode.RobotVars.SHITTY_WORKAROUND_TIME;
import static org.firstinspires.ftc.teamcode.RobotVars.SHP;
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
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.extA;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.extB;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.imu;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.initma;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.leftBack;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.leftFront;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.rid;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.ridA;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.ridB;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.rightBack;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.rightFront;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.rpd;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sBalans;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sClose;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sHeading;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sMCLaw;
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
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

    public static double P1H = 0.541;
    public static double P1X = -138;
    public static double P1Y = -11;
    public static double P2H = 1.431;
    public static double P2X = -123;
    public static double P2Y = -14;
    public static double P3H = 1.92;
    public static double P3X = -112;
    public static double P3Y = -71;
    public static double P4H = 1.40;
    public static double P4X = -120;
    public static double P4Y = -15;

    public static double OFFX = 2;
    public static double OFFY = 2;
    public static double OFFH = -0.01;

    public static double OFFX1 = 2;
    public static double OFFY1 = 0.2;
    public static double OFFH1 = -0.002;

    public static double P678X = -118;
    public static double P678H = 1.431;
    public static double P6Y = -58;
    public static double P7Y = 4;
    public static double P8Y = 60;

    public static double PTG1X = 15.0;
    public static double PTG1Y = 1.5;
    public static double PTG2X = 15.0;
    public static double PTG2Y = 0.5;

    public static double RAI1X = 20;
    public static double RAI1Y = 4;
    public static double RAI2X = 30;
    public static double RAI2Y = 3.2;

    public static double RBI1X = 20;
    public static double RBI1Y = 4;
    public static double RBI2X = 30;
    public static double RBI2Y = 3.2;

    public static boolean BBBBBBBBBBBBBB = true;
    public static boolean RECURRING_SINGULARITY = false;
    public static boolean GPOS = false;

    /*public static double MVEL = 250;
    public static double MAL = 250;
    public static double MDL = 100;*/
    public static double MVEL = 999; // THE ONLY TYPE OF
    public static double MAL = 999;  // BREAKING I KNOW IS
    public static double MDL = 999;  // BREAKING BAD

    public double WD = 0.02;
    public static double WD2 = 0.3;

    public static double R1X = 40;
    public static double R1Y = 2.4;
    public static double R2X = 40;
    public static double R2Y = 1.2;

    Vector<Double> v = new Vector<>();
    Vector<Pose2d> e = new Vector<>();
    double it;

    void ltime() {
        v.add(getRuntime() - it);
        e.add(drive.getLastError());
    }

    class Spike {
        double time;
        String name;
        double val;

        public Spike(double v, String n) {
            this.time = getRuntime();
            this.val = v;
            this.name = n;
        }
    }

    void set_wait_time(double t) {
        drive.follower = new HolonomicPIDVAFollower(SampleMecanumDrive.AXIAL_PID, SampleMecanumDrive.LATERAL_PID, SampleMecanumDrive.HEADING_PID, new Pose2d(2, 2, Math.toRadians(2)), t);
        drive.trajectorySequenceRunner = new TrajectorySequenceRunner(drive.follower, SampleMecanumDrive.HEADING_PID);
    }

    Vector<Spike> spv = new Vector<>();

    public static double SPIKE_THRESHOLD = 20000;

    void chenc(double ol, double ne, String name) {
        if ((ne < 0 && ol < 0) && ((ne - ol) > SPIKE_THRESHOLD)) {
            spv.add(new Spike(ne - ol, name));
        } else if ((ne > 0 && ol > 0) && ((ol - ne) > SPIKE_THRESHOLD)) {
            spv.add(new Spike(ne - ol, name));
        }
    }

    void follow_traj(TrajectorySequence traj) {
        drive.followTrajectorySequenceAsync(traj);
        drive.update();
        TelemetryPacket pack;
        ElapsedTime timer = new ElapsedTime(0);
        double lev = leftEncoder.getCorrectedVelocity();
        double rev = rightEncoder.getCorrectedVelocity();
        double fev = frontEncoder.getCorrectedVelocity();
        double lcv;
        double rcv;
        double fcv;
        while (drive.isBusy() && !isStopRequested() && traj != null && !gamepad1.right_bumper) {
            drive.update();
            lcv = leftEncoder.getCorrectedVelocity();
            rcv = rightEncoder.getCorrectedVelocity();
            fcv = frontEncoder.getCorrectedVelocity();
            chenc(lev, lcv, "Left");
            chenc(rev, rcv, "Right");
            chenc(fev, fcv, "Front");
            lev = lcv;
            rev = rcv;
            fev = fcv;
            for (Spike s : spv) {
                telemetry.addLine("Caught spike " + s.name + " at " + s.time + "of" + s.val + "!");
            }
            telemetry.update();
            leftEncoder.getCorrectedVelocity();
            pack = new TelemetryPacket();
            pack.put("Ex", drive.getLastError().getX());
            pack.put("Ey", drive.getLastError().getY());
            pack.put("Eh", drive.getLastError().getHeading());
            pack.put("CycleTime", timer.milliseconds());
            pack.put("vel", leftEncoder.getCorrectedVelocity());
            pack.put("ver", rightEncoder.getCorrectedVelocity());
            pack.put("vef", frontEncoder.getCorrectedVelocity());
            if (extA != null) {
                pack.put("CUR_extA", extA.getCurrent(CurrentUnit.MILLIAMPS));
                pack.put("CUR_extB", extB.getCurrent(CurrentUnit.MILLIAMPS));
                pack.put("POW_extA", extA.getPower());
                pack.put("POW_extB", extB.getPower());
            }
            if (ridA != null) {
                pack.put("CUR_ridA", ridA.getCurrent(CurrentUnit.MILLIAMPS));
                pack.put("CUR_ridB", ridB.getCurrent(CurrentUnit.MILLIAMPS));
                pack.put("POW_ridA", ridA.getPower());
                pack.put("POW_ridB", ridB.getPower());
            }
            timer.reset();
            dashboard.sendTelemetryPacket(pack);

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
    public static double HT = 0.13;
    public static double HHT = 0.12;

    public static double WHO = 0.8;
    public static double WEX = 0.8;

    int lp = 1;
    public static double SGS = 0.495;
    public static double SGD = 0.024;
    public static double SBAS = 0.60;
    public static double SBAD = 0.0;

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
        clo.toPut = false;
    }

    void set_grab_pos(int p) {
        conversiePerverssa(SGS - SGD * (p - 1));
        sBalans.setPosition(SBAS - SBAD * (p - 1));
    }

    void upd_grab_pos() {
        set_grab_pos(lp);
        ++lp;
    }

    public static int NUMC = 5;

    TrajectorySequence goToPreload;
    TrajectorySequence preloadToGet;
    TrajectorySequence startGrab;
    TrajectorySequence goToPark;
    TrajectorySequence ender;
    Vector<Vector<TrajectorySequence>> trss = new Vector<>();

    IchHasseKinder ihk;
    Thread ihkT;

    void mktraj() {
        Vector2d RAI1 = new Vector2d(RAI1X, RAI1Y);
        Vector2d RAI2 = new Vector2d(RAI2X, RAI2Y);
        Vector2d RBI1 = new Vector2d(RBI1X, RBI1Y);
        Vector2d RBI2 = new Vector2d(RBI2X, RBI2Y);
        Vector2d R1 = new Vector2d(R1X, R1Y);
        Vector2d R2 = new Vector2d(R2X, R2Y);
        Vector2d PTG1 = new Vector2d(PTG1X, PTG1Y);
        Vector2d PTG2 = new Vector2d(PTG2X, PTG2Y);
        TrajectoryVelocityConstraint vc = SampleMecanumDrive.getVelocityConstraint(MVEL, MAX_ANG_VEL, TRACK_WIDTH);
        TrajectoryAccelerationConstraint ac = SampleMecanumDrive.getAccelerationConstraint(MAL);
        TrajectoryAccelerationConstraint dc = SampleMecanumDrive.getAccelerationConstraint(MDL);
        lp = 1;

        goToPreload = drive.trajectorySequenceBuilder(new Pose2d(SPOSX, SPOSY, SPOSH))
                .funnyRaikuCurveLinear(new Pose2d(P1X, P1Y, P1H), R1, R2, vc, ac, dc)
                .UNSTABLE_addTemporalMarkerOffset(RD, () -> rid(RTOP_POS))
                .UNSTABLE_addTemporalMarkerOffset(0.0, this::ltime)
                .waitSeconds(WD)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    rid(RBOT_POS);
                    ret();
                    set_grab_pos(1);
                })
                .build();

        preloadToGet = drive.trajectorySequenceBuilder(new Pose2d(P1X + 0.00001, P1Y, P1H))
                .funnyRaikuCurveLinear(new Pose2d(P2X, P2Y, P2H), PTG1, PTG2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    epd.set_target(EMAX, 0);
                    upd_grab_pos();
                })
                .build();

        startGrab = drive.trajectorySequenceBuilder(new Pose2d(P2X + 0.0001, P2Y, P2H))
                .lineToLinearHeading(new Pose2d(P2X, P2Y, P2H))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> sClose.setPosition(SINCHIS))
                .waitSeconds(0.11)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    conversiePerverssa(SAH);
                    sBalans.setPosition(SBAH);
                })
                .waitSeconds(HT)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    sHeading.setPosition(SHP);
                    sBalans.setPosition(SBAP);
                    conversiePerverssa(SAP);
                })
                .waitSeconds(HHT)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> ihk.stage = 1)
                .build();

        for (int i = 0; i < NUMC; ++i) {
            Vector<TrajectorySequence> trs = new Vector<>();
            trs.add(drive.trajectorySequenceBuilder(new Pose2d(P4X + OFFX * i, P4Y + OFFY * i, P4H + OFFH * i)) /// From Get cone to stalp
                    .funnyRaikuCurveLinear(new Pose2d(P3X + OFFX1 * i, P3Y + OFFY1 * i, P3H + OFFH1 * i), RAI1, RAI2)
                    .build());

            trs.add(drive.trajectorySequenceBuilder(new Pose2d(P3X + OFFX1 * i, P3Y + OFFY1 * i, P3H + OFFH1 * i)) // Retract, extend and go to ext
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        rid(RBOT_POS);
                        ret();
                        epd.set_target(EMAX, 0);
                        upd_grab_pos();
                    })
                    .funnyRaikuCurveLinear(new Pose2d(P4X + OFFX * i, P4Y + OFFY * i, P4H + OFFH * i), RBI1, RBI2)
                    .build());

            trs.add(drive.trajectorySequenceBuilder(new Pose2d(P4X + OFFX * i + 0.0001, P4Y + OFFY * i, P4H + OFFH * i)) // At get retract
                    .lineToLinearHeading(new Pose2d(P4X + OFFX * i, P4Y + OFFY * i, P4H + OFFH * i))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> sClose.setPosition(SINCHIS))
                    .waitSeconds(0.11)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        conversiePerverssa(SAH);
                        sBalans.setPosition(SBAH);
                    })
                    .waitSeconds(HT)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        sHeading.setPosition(SHP);
                        sBalans.setPosition(SBAP);
                        conversiePerverssa(SAP);
                    })
                    .waitSeconds(HHT)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ihk.stage = 1)
                    .build());

            trss.add(trs);
        }

        ender = drive.trajectorySequenceBuilder(new Pose2d(P3X + OFFX1 * NUMC + 0.0001, P3Y + OFFY1 * NUMC, P3H + OFFH1 * NUMC))
                .lineToLinearHeading(new Pose2d(P3X + OFFX1 * NUMC, P3Y + OFFY1 * NUMC, P3H + OFFH1 * NUMC))
                .addTemporalMarker(this::ret)
                .waitSeconds(1)
                .build();

        switch (LAST_ID) {
            default:
            case 7:
                goToPark = drive.trajectorySequenceBuilder(new Pose2d(P2X, P2Y, P2H))
                        .addTemporalMarker(() -> {
                            conversiePerverssa(SAW);
                            sClose.setPosition(SDESCHIS);
                            sHeading.setPosition(SHG);
                            sBalans.setPosition(SBAG);
                            ext(EMIN);
                        })
                        .lineToLinearHeading(new Pose2d(P678X, P7Y, P678H))
                        .addTemporalMarker(this::ltime)
                        .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> ext(EMIN))
                        .waitSeconds(1)
                        .build();
                break;
            case 6:
                goToPark = drive.trajectorySequenceBuilder(new Pose2d(P2X, P2Y, P2H))
                        .addTemporalMarker(() -> {
                            conversiePerverssa(SAW);
                            sClose.setPosition(SDESCHIS);
                            sHeading.setPosition(SHG);
                            sBalans.setPosition(SBAG);
                            ext(EMIN);
                        })
                        .lineToLinearHeading(new Pose2d(P678X, P6Y, P678H))
                        .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> ext(EMIN))
                        .addTemporalMarker(this::ltime)
                        .waitSeconds(1)
                        .build();
                break;
            case 8:
                goToPark = drive.trajectorySequenceBuilder(new Pose2d(P2X, P2Y, P2H))
                        .addTemporalMarker(() -> {
                            conversiePerverssa(SAW);
                            sClose.setPosition(SDESCHIS);
                            sHeading.setPosition(SHG);
                            sBalans.setPosition(SBAG);
                            ext(EMIN);
                        })
                        .lineToLinearHeading(new Pose2d(P678X, P8Y, P678H))
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
        boolean SGP = false;
        boolean DL = false;
        boolean DR = false;
        boolean DU = false;
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
                if (extA != null) {
                    p.put("CUR_extA", extA.getCurrent(CurrentUnit.MILLIAMPS));
                    p.put("CUR_extB", extB.getCurrent(CurrentUnit.MILLIAMPS));
                    p.put("POW_extA", extA.getPower());
                    p.put("POW_extB", extB.getPower());
                }
                if (ridA != null) {
                    p.put("CUR_ridA", ridA.getCurrent(CurrentUnit.MILLIAMPS));
                    p.put("CUR_ridB", ridB.getCurrent(CurrentUnit.MILLIAMPS));
                    p.put("POW_ridA", ridA.getPower());
                    p.put("POW_ridB", ridB.getPower());
                }
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
                    sClose.setPosition(SDESCHIS);
                    epd.set_target(EMAX, 0);
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
        drive.setPoseEstimate(new Pose2d(cp.getX(), cp.getY(), P2H));
    }

    Encoder leftEncoder, rightEncoder, frontEncoder;
    ElapsedTime SHITTY_WORKAROUND_TIMER = new ElapsedTime(0);
    boolean SHITTY_WORKAROUND_TIMED = false;

    public static double WAT = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        initma(hardwareMap);
        ihk = new IchHasseKinder();
        ihkT = new Thread(ihk);
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

        @SuppressLint("DiscouragedApi") int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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
        ihk.shouldClose = false;
        ihk.lom = this;
        ihkT.start();

        drive.setPoseEstimate(new Pose2d(SPOSX, SPOSY, SPOSH));
        //drive.setPoseEstimate(new Pose2d(0, 0, 0));

        if (!BBBBBBBBBBBBBB) {
            clo.shouldClose = true;
            rpd.shouldClose = true;
            epd.shouldClose = true;
            telemetry.addLine("Start");
            telemetry.update();
            double P11X = 0, P11Y = 0, P12X = 0, P12Y = 0;

            while (!isStopRequested()) {
                if (P11X != RAI1X || P12X != RAI2X || P11Y != RAI1Y || P12Y != RAI2Y) {
                    mktraj();
                    if (goToPreload == null) {
                        continue;
                    }
                    TelemetryPacket p = new TelemetryPacket();
                    Canvas fieldOverlay = p.fieldOverlay();
                    draw(fieldOverlay, goToPreload);
                    draw(fieldOverlay, preloadToGet);
                    draw(fieldOverlay, trss.get(0).get(0));
                    draw(fieldOverlay, trss.get(0).get(1));
                    draw(fieldOverlay, trss.get(0).get(2));
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
        } else {
            packet = new TelemetryPacket();
            packet.put("LID", LAST_ID);
            dashboard.sendTelemetryPacket(packet);

            it = getRuntime();

            SHITTY_WORKAROUND_TIMER.reset();
            SHITTY_WORKAROUND_TIMED = false;
            getpos();
            set_wait_time(0);
            follow_traj(goToPreload);
            getpos();
            set_wait_time(WAT);
            follow_traj(preloadToGet);
            wtfor(RobotFuncs.WAITS.EXTENSION, WEX);
            set_wait_time(0);
            follow_traj(startGrab);
            getpos();
            for (int i = 0; i < NUMC - 1; ++i) {
                follow_traj(trss.get(i).get(0)); // Go to stalp
                wtfor(RobotFuncs.WAITS.HOISTER, WHO);
                set_wait_time(WAT);
                follow_traj(trss.get(i).get(1)); // Retract, Extend and go to get
                wtfor(RobotFuncs.WAITS.EXTENSION, WEX);
                set_wait_time(0);
                follow_traj(trss.get(i).get(2)); // At get (retract)
                getpos();
            }

            if (NUMC >= 1) {
                set_wait_time(0);
                follow_traj(trss.get(NUMC - 1).get(0)); // Go to stalp
                wtfor(RobotFuncs.WAITS.HOISTER, WHO);
                rid(RBOT_POS);
                ret();
                wtfor(RobotFuncs.WAITS.HOISTER_FALL, 0);
            }
            getpos();

            if (RECURRING_SINGULARITY && !isStopRequested()) {
                TrajectorySequence traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .addDisplacementMarker(() -> ext(EMIN))
                        .lineToLinearHeading(new Pose2d(0, 0, 0))
                        .build();
                follow_traj(traj);
            } else if (!isStopRequested()) {
                follow_traj(ender);
            }

            endma();
            ihk.shouldClose = true;

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

            packet = new TelemetryPacket();
            for (Spike s : spv) {
                packet.addLine("Caught spike " + s.name + " at " + s.time + " of " + s.val + "!");
                telemetry.speak("SPIKE DETECTED " + s.name);
            }
            dashboard.sendTelemetryPacket(packet);

        }
    }
}
