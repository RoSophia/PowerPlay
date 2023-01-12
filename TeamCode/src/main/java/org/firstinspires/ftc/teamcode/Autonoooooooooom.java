package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayList;
import java.util.Vector;

@Config
@Autonomous(group = "drive")
@SuppressLint("DefaultLocale")//
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

    public static int F = 65;

    double S1CL = 0.33;
    double S1OP = 0.75;

    public static double HEAD1 = Math.toRadians(58.56);
    public static double PX1 = 160;
    public static double PY1 = -3.4;
    public static double HEAD2 = Math.toRadians(270);
    public static double PX2 = 136;
    public static double PY2 = -51;
    public static double HEAD3 = Math.toRadians(50.592);
    public static double PX3 = 154.93;
    public static double PY3 = -0.82;
    public static double HEADC = Math.toRadians(-1);

    public static boolean AAAAAAAAAAAAAA = false;
    public static boolean RECURRING_SINGULARITY = true;
    boolean OPENED = false;

    public static double MVEL = 165;
    public static double MAL  = 165;

    VoltageSensor batteryVoltageSensor;

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

    @Override
    public void runOpMode() throws InterruptedException {
        Servo s1 = hardwareMap.get(Servo.class, "S1");
        DcMotor ridicareSlide = hardwareMap.get(DcMotor.class, "RS");
        ridicareSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ridicareSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        ridicareSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        s1.setPosition(S1CL);

        drive = new SampleMecanumDrive(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        TelemetryPacket packet;

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
        }

        waitForStart();

        ridicareSlide.setPower(0.5);
        ridicareSlide.setTargetPosition(40);
        ridicareSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        webcam.closeCameraDeviceAsync(() -> {});

        TelemetryPacket pack = new TelemetryPacket();
        pack.put("bat", batteryVoltageSensor.getVoltage());
        dashboard.sendTelemetryPacket(pack);

        TrajectorySequence traj = null;
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
            /*while (!isStopRequested()) {
                drive.updatePoseEstimate();
                packet = new TelemetryPacket();
                packet.put("px", drive.getPoseEstimate().getX());
                packet.put("py", drive.getPoseEstimate().getY());
                packet.put("ph", drive.getPoseEstimate().getHeading());
                dashboard.sendTelemetryPacket(packet);
            }*/
        } else {
            packet = new TelemetryPacket();
            packet.put("LID", LAST_ID);
            dashboard.sendTelemetryPacket(packet);

            TrajectoryVelocityConstraint vc = SampleMecanumDrive.getVelocityConstraint(MVEL, MAX_ANG_VEL, TRACK_WIDTH);
            TrajectoryAccelerationConstraint ac = SampleMecanumDrive.getAccelerationConstraint(MAL);

            traj = drive.trajectorySequenceBuilder(new Pose2d())
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        ridicareSlide.setPower(0.4);
                        ridicareSlide.setTargetPosition(100);
                    })
                    .lineToLinearHeading(new Pose2d(F * 2 - 10, -10, 0), vc, ac)
                    .UNSTABLE_addTemporalMarkerOffset(0.18, () -> {
                        ridicareSlide.setPower(1);
                        ridicareSlide.setTargetPosition(TOP_POS);
                        s1.setPosition(S1CL);
                    })
                    .lineToLinearHeading(new Pose2d(PX1, PY1, HEAD1))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ridicareSlide.setTargetPosition(TOP_POS - 150))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> s1.setPosition(S1OP)) ///////////////////////////// PRELOAD 1
                    .lineToLinearHeading(new Pose2d(F * 2 - 10, 0, Math.toRadians(-30)))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        ridicareSlide.setPower(0.4);
                        ridicareSlide.setTargetPosition(360);
                    })
                    .lineToLinearHeading(new Pose2d(PX2, PY2, HEAD2)) //////////////////////////////////////////// GET CONE 1
                    .UNSTABLE_addTemporalMarkerOffset(-0.03, () -> s1.setPosition(S1CL))
                    .waitSeconds(0.12)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        ridicareSlide.setPower(1);
                        ridicareSlide.setTargetPosition(TOP_POS / 2);
                    })
                    .lineToLinearHeading(new Pose2d(F * 2, 0, 0))
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> ridicareSlide.setTargetPosition(TOP_POS))
                    .lineToLinearHeading(new Pose2d(PX3, PY3, HEAD3))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ridicareSlide.setTargetPosition(TOP_POS - 150))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> s1.setPosition(S1OP)) ///////////////////////////// CONE 1
                    .lineToLinearHeading(new Pose2d(F * 2 - 10, 0, Math.toRadians(-30)))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        ridicareSlide.setPower(0.4);
                        ridicareSlide.setTargetPosition(310);
                    })
                    .lineToLinearHeading(new Pose2d(PX2, PY2, HEAD2)) //////////////////////////////////////////// GET CONE 2
                    .UNSTABLE_addTemporalMarkerOffset(-0.07, () -> s1.setPosition(S1CL))
                    .waitSeconds(0.1)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        ridicareSlide.setPower(1);
                        ridicareSlide.setTargetPosition(TOP_POS / 2);
                    })
                    .lineToLinearHeading(new Pose2d(F * 2, 0, 0))
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> ridicareSlide.setTargetPosition(TOP_POS))
                    .lineToLinearHeading(new Pose2d(PX3, PY3, HEAD3 + HEADC))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ridicareSlide.setTargetPosition(TOP_POS - 150))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> s1.setPosition(S1OP)) ///////////////////////////// CONE 2
                    .lineToLinearHeading(new Pose2d(F * 2 - 10, 0, Math.toRadians(-30)))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        ridicareSlide.setPower(0.4);
                        ridicareSlide.setTargetPosition(260);
                    })
                    .lineToLinearHeading(new Pose2d(PX2, PY2, HEAD2)) //////////////////////////////////////////// GET CONE 3
                    .UNSTABLE_addTemporalMarkerOffset(-0.07, () -> s1.setPosition(S1CL))
                    .waitSeconds(0.1)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        ridicareSlide.setPower(1);
                        ridicareSlide.setTargetPosition(TOP_POS / 2);
                    })
                    .lineToLinearHeading(new Pose2d(F * 2, 0, 0))
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> ridicareSlide.setTargetPosition(TOP_POS))
                    .lineToLinearHeading(new Pose2d(PX3, PY3, HEAD3 + HEADC * 2))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> ridicareSlide.setTargetPosition(TOP_POS - 150))
                    .waitSeconds(0.1)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> s1.setPosition(S1OP)) ///////////////////////////// CONE 3



                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        ridicareSlide.setPower(0.4);
                        ridicareSlide.setTargetPosition(70);
                    })
                    .build();
            follow_traj(traj);

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
                    .lineToLinearHeading(new Pose2d(0, 0, 0))
                    .build();
            follow_traj(traj);
        }
    }
}