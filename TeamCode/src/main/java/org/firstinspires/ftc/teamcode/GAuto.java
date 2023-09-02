package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.BOT_POS;
import static org.firstinspires.ftc.teamcode.RobotConstants.DOT;
import static org.firstinspires.ftc.teamcode.RobotConstants.SDESCHIS;
import static org.firstinspires.ftc.teamcode.RobotConstants.SINCHIS;
import static org.firstinspires.ftc.teamcode.RobotConstants.TOP_POS;
import static org.firstinspires.ftc.teamcode.RobotConstants.UPT;
import static org.firstinspires.ftc.teamcode.RobotConstants.pcoef;
import static org.firstinspires.ftc.teamcode.RobotFuncs.armRun;
import static org.firstinspires.ftc.teamcode.RobotFuncs.batteryVoltageSensor;
import static org.firstinspires.ftc.teamcode.RobotFuncs.dashboard;
import static org.firstinspires.ftc.teamcode.RobotFuncs.endma;
import static org.firstinspires.ftc.teamcode.RobotFuncs.imu;
import static org.firstinspires.ftc.teamcode.RobotFuncs.initma;
import static org.firstinspires.ftc.teamcode.RobotFuncs.leftBack;
import static org.firstinspires.ftc.teamcode.RobotFuncs.leftFront;
import static org.firstinspires.ftc.teamcode.RobotFuncs.ridicareSlide;
import static org.firstinspires.ftc.teamcode.RobotFuncs.rightBack;
import static org.firstinspires.ftc.teamcode.RobotFuncs.rightFront;
import static org.firstinspires.ftc.teamcode.RobotFuncs.s1;
import static org.firstinspires.ftc.teamcode.RobotFuncs.startma;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
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
import java.util.Arrays;
import java.util.List;
import java.util.Vector;

@SuppressWarnings("CommentedOutCode")
@Config
@Autonomous(group = "drive")
@SuppressLint("DefaultLocale")
public class GAuto extends LinearOpMode {
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

    public static double HEAD1 = 5.543;
    public static double PX1 = 158;
    public static double PY1 = -4.0;
    public static double HEAD2 = -4.7;
    public static double PX2 = 147;
    public static double PY2 = 60.5;
    public static double HEAD3 = 5.84;
    public static double PX3 = 150;
    public static double PY3 = -9;
    public static double HEADC = -0.01;
    public static double HEADCVC = 0.00;
    public static double HEADCC = -0.02;
    public static double PXXC = 1.35;
    public static double PXC = -0.3;
    public static double PYC = -0.6;
    public static double PYYC = 0.3;
    public static int ADIF = 350;

    public static double P1X = 35;
    public static double P1Y = -3.0;
    public static double P2X = 25;
    public static double P2Y = -2;

    public static boolean AAAAAAAAAAAAAA = false;
    public static boolean BBBBBBBBBBBBBB = true;
    public static boolean RECURRING_SINGULARITY = false;
    public static boolean GPOS = false;

    public static double MVEL = 160;//150;
    public static double MAL = 120;//100;
    public static double MDL = 70;//70;

    public double OPD = 0.04;
    public double UPD = 0.7;
    public static double WTD = 1;
    public double WD = 0.02;
    public double WWD = 0.08;
    public static double PD = 0.3;

    public static double R1X = 30;
    public static double R1Y = -0.1;
    public static double R1YVC = 0.09;
    public static double R2X = 40;
    public static double R2Y = -4;

    public double H11 = -0.8;
    public double H12 = 0.8;
    public double H21 = -1;
    public double H22 = 1;

    List<Integer> GP = Arrays.asList(245, 195, 143, 100, 20);

    Vector<Double> v = new Vector<>();
    Vector<Pose2d> e = new Vector<>();
    double it;

    void ltime() {
        updh();
        v.add(getRuntime() - it);
        e.add(drive.getLastError());
    }

    Encoder frontEncoder;
    Encoder rightEncoder;
    Encoder leftEncoder;

    void follow_traj(TrajectorySequence traj) {
        drive.followTrajectorySequenceAsync(traj);
        drive.update();
        TelemetryPacket pack;
        ElapsedTime timer = new ElapsedTime(0);
        while (drive.isBusy() && !isStopRequested() && traj != null) {
            drive.update();
            pack = new TelemetryPacket();
            pack.put("Ex", drive.getLastError().getX());
            pack.put("Ey", drive.getLastError().getY());
            pack.put("Eh", drive.getLastError().getHeading());
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

    int c = 0;

    void sp() {
        armRun.set_target(GP.get(c), DOT);
        ++c;
    }

    /*
    public static boolean KOOKY = false;
    boolean PARK = false;
    void vc() {
        if (KOOKY) {
            final double cd = cs.getDistance(DistanceUnit.MM);
            if (cd > 100) {
                PARK = true;
                armRun.set_target(TOP_POS / 2, UPT);
            }
        }
    }*/

    public static int NCON = 0;

    @SuppressWarnings("ConstantConditions")
    TrajectorySequence mktraj() {
        if (false) {
            TrajectoryVelocityConstraint vc = SampleMecanumDrive.getVelocityConstraint(MVEL, MAX_ANG_VEL, TRACK_WIDTH);
            TrajectoryAccelerationConstraint ac = SampleMecanumDrive.getAccelerationConstraint(MAL);
            return drive.trajectorySequenceBuilder(new Pose2d())
                    .lineToConstantHeading(new Vector2d(F * 2, 0), vc, ac)
                    .build();
        } else {
            Vector2d P1 = new Vector2d(P1X, P1Y);
            Vector2d P2 = new Vector2d(P2X, P2Y);
            double R1YVCC = (13.4 - batteryVoltageSensor.getVoltage()) * R1YVC;
            Vector2d R1 = new Vector2d(R1X, R1Y + R1YVCC);
            Vector2d R2 = new Vector2d(R2X, R2Y);
            TrajectoryVelocityConstraint vc = SampleMecanumDrive.getVelocityConstraint(MVEL, MAX_ANG_VEL, TRACK_WIDTH);
            TrajectoryAccelerationConstraint ac = SampleMecanumDrive.getAccelerationConstraint(MAL);
            TrajectoryAccelerationConstraint dc = SampleMecanumDrive.getAccelerationConstraint(MDL);
            double HEADCVCC = (13.4 - batteryVoltageSensor.getVoltage()) * HEADCVC;
            TrajectorySequenceBuilder cs = drive.trajectorySequenceBuilder(new Pose2d(SPOSX, SPOSY, SPOSH))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> armRun.set_target(100, 0.5))
                    .UNSTABLE_addTemporalMarkerOffset(WTD, () -> {
                        armRun.set_target(TOP_POS, UPT);
                        s1.setPosition(SINCHIS);
                    })
                    .funnyRaikuCurve(new Pose2d(PX1, PY1, HEAD1), R1, R2, H11, H12, vc, ac, dc)
                    .addTemporalMarker(this::ltime)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        getpos();
                        armRun.set_target(TOP_POS - ADIF, 0);
                    })
                    .waitSeconds(WD)
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s1.setPosition(SDESCHIS)); ///////////////////////////// PRELOAD 1
            for (int i = 0; i < NCON; ++i) {
                cs.UNSTABLE_addTemporalMarkerOffset(PD, this::sp)
                        .funnyRaikuCurve(new Pose2d(PX2 + PXXC * i, PY2 + PYYC * i, HEAD2 + HEADCC * i), P1, P2, H21, H22) //////////////////////////////////////////// GET CONE 1
                        .addTemporalMarker(this::ltime)
                        .UNSTABLE_addTemporalMarkerOffset(-0.03, () -> {
                            getpos();
                            s1.setPosition(SINCHIS);
                        })
                        .waitSeconds(WWD)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            armRun.set_target(TOP_POS / 2, UPT / 2);
                            //vc();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(UPD, () -> armRun.set_target(TOP_POS, UPT))
                        .funnyRaikuCurve(new Pose2d(PX3 + PXC * i, PY3 + PYC * i, HEAD3 + HEADC * i + HEADCVCC * i), P2, P1, H21, H22)
                        .addTemporalMarker(this::ltime)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            getpos();
                            armRun.set_target(TOP_POS - ADIF, 0);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(OPD, () -> s1.setPosition(SDESCHIS)); ///////////////////////////// CONE 1
            }
            return cs.waitSeconds(0.2)
                    .build();
        }
    }

    boolean OPENED = false;
    boolean NB = false;
    boolean CL = false;

    void getpos() {
        if (GPOS) {
            while (!isStopRequested()) {
                drive.updatePoseEstimate();
                telemetry.addData("PE", drive.getPoseEstimate());
                telemetry.addData("PEH", drive.getPoseEstimate().getHeading() * Math.PI / 180);
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
                pcoef = 14.0 / batteryVoltageSensor.getVoltage();
                final double spcoef = 1 - 0.6 * gamepad1.right_trigger;
                final double fcoef = pcoef * spcoef * 0.7;
                leftFront.setPower(lfPower * fcoef);
                rightFront.setPower(rfPower * fcoef);
                leftBack.setPower(lbPower * fcoef);
                rightBack.setPower(rbPower * fcoef);

                if (gamepad1.a) {
                    break;
                }
                if (gamepad1.b && !NB) {
                    armRun.set_target(BOT_POS, DOT * 2);
                    NB = true;
                }
                if (gamepad1.x && !CL) {
                    s1.setPosition(SDESCHIS);
                    CL = true;
                }
            }
        }
    }

    void updh() {
        Pose2d cp = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(cp.getX(), cp.getY(), imu.getAngularOrientation().firstAngle));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initma(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(SPOSX, SPOSY, SPOSH));
        s1.setPosition(SINCHIS);

        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "LED"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "Underglow"));
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "LB"));

        TelemetryPacket packet;

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

        TrajectorySequence traj = null;

        if (BBBBBBBBBBBBBB) {
            traj = mktraj();
        }

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
            sleep(2);
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
        startma(this);

        drive.setPoseEstimate(new Pose2d(SPOSX, SPOSY, SPOSH));
        //drive.setPoseEstimate(new Pose2d(0, 0, 0));

        if (!BBBBBBBBBBBBBB) {
            telemetry.addLine("Start");
            telemetry.update();
            double P11X = 0, P11Y = 0, P12X = 0, P12Y = 0;
            double R11X = 0, R11Y = 0, R12X = 0, R12Y = 0;

            while (!isStopRequested()) {
                if (P11X != PX1 || P12X != PX2 || P11Y != PY1 || P12Y != PY2 || R11X != R1X || R11Y != R1Y || R12X != R2X || R12Y != R2Y) {
                    traj = mktraj();
                    TelemetryPacket p = new TelemetryPacket();
                    Canvas fieldOverlay = p.fieldOverlay();
                    draw(fieldOverlay, traj);
                    p.put("Updated!", 0);
                    p.put("StartX", traj.start().getX());
                    p.put("StartY", traj.start().getY());
                    p.put("StartH", traj.start().getY());
                    p.put("EndX", traj.start().getX());
                    p.put("EndY", traj.start().getY());
                    p.put("EndH", traj.start().getY());
                    dashboard.sendTelemetryPacket(p);
                    P11X = P1X;
                    P11Y = P1Y;
                    P12X = P2X;
                    P12Y = P2Y;
                    R11X = R1X;
                    R11Y = R1Y;
                    R12X = R2X;
                    R12Y = R2Y;
                }

                sleep(10);
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

                it = getRuntime();
                follow_traj(traj);

                if (!isStopRequested()) {
                    switch (LAST_ID) {
                        default:
                            telemetry.addLine("DEFAULT");
                        case 7:
                            telemetry.addLine("6");
                            traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(F * 2, 6, 0))
                                    .addTemporalMarker(this::ltime)
                                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> armRun.set_target(70, 1))
                                    .waitSeconds(1.5)
                                    .build();
                            break;
                        case 6:
                            telemetry.addLine("7");
                            traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(F * 2, F * 0.95, 0))
                                    .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> armRun.set_target(70, 1))
                                    .addTemporalMarker(this::ltime)
                                    .waitSeconds(1.5)
                                    .build();
                            break;
                        case 8:
                            telemetry.addLine("8");
                            traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    //.funnyRaikuCurve(new Pose2d(F * 2, -F * 1.2, 0), new Vector2d(20, 4.5), new Vector2d(1, 0), H21, H11)
                                    .lineToLinearHeading(new Pose2d(F * 2, -16, 0))
                                    .lineToLinearHeading(new Pose2d(F * 2, -F, 0))
                                    .addTemporalMarker(this::ltime)
                                    .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> armRun.set_target(70, 1))
                                    .waitSeconds(1.5)
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
                        .addDisplacementMarker(() -> {
                            ridicareSlide.setPower(0.2);
                            ridicareSlide.setTargetPosition(100);
                        })
                        .lineToLinearHeading(new Pose2d(0, 0, 0))
                        .build();
                follow_traj(traj);
            }
        }

        endma();

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
