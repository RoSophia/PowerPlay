package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
    /*final double FX = 578.272;
    final double FY = 578.272;*/
    final double FX = 878.272;
    final double FY = 878.272;
    final double CX = 320;
    final double CY = 240;
    public static int LL = 0;

    final double PI2 = Math.PI / 2;
    final double PI = Math.PI;

    static final double FEET_PER_METER = 3.28084;

    int LAST_ID = 0;

    public int TOP_POS = 1303;
    public int MIU_POS = 1010;
    public int MID_POS = 760;

    public static int NORMAL = 0;

    public static int F = 65;

    double S1CL = 0.33;
    double S1OP = 0.75;

    public static double HEAD1 = Math.toRadians(311);
    public static double PX1 = 142;
    public static double PY1 = 43;
    public static double HEAD2 = Math.toRadians(270);
    public static double PX2 = 132;
    public static double PY2 = -53;
    public static double HEAD3 = Math.toRadians(388.5);
    public static double PX3 = 144.5;
    public static double PY3 = 8;

    public static boolean AAAAAAAAAAAAAA = false;
    boolean OPENED = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo s1 = hardwareMap.get(Servo.class, "S1");
        DcMotor ridicareSlide = hardwareMap.get(DcMotor.class, "RS");
        ridicareSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ridicareSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        ridicareSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        s1.setPosition(S1CL);

        drive = new SampleMecanumDrive(hardwareMap);

        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

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
        webcam.closeCameraDeviceAsync(() -> {
        });

        TrajectorySequence ct;
        if (AAAAAAAAAAAAAA) {
            while (!isStopRequested()) {
                if (!drive.isBusy()) {
                    ct = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(0, -F, 0))
                            .lineToLinearHeading(new Pose2d(0, F, 0))
                            .build();
                    drive.followTrajectorySequenceAsync(ct);
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

            TrajectorySequence traj = null;
            if (NORMAL == 0) {
                if (LL != 0) {
                    LAST_ID += LL + 5;
                }

                switch (LAST_ID) {
                    case 6:
                        //traj = drive.trajectorySequenceBuilder(new Pose2d())
                        traj = drive.trajectorySequenceBuilder(new Pose2d())
                                //traj = drive.trajectorySequenceBuilder(new Pose2d())
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    ridicareSlide.setPower(0.4);
                                    ridicareSlide.setTargetPosition(100);
                                })
                                .lineToLinearHeading(new Pose2d(8, F - 7, 0))
                                .lineToLinearHeading(new Pose2d(F * 2 - 7, F - 7, 0))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    ridicareSlide.setPower(0.7);
                                    ridicareSlide.setTargetPosition(TOP_POS);
                                    s1.setPosition(S1CL);
                                })
                                .lineToLinearHeading(new Pose2d(PX1, PY1, HEAD1))
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> ridicareSlide.setTargetPosition(TOP_POS - 150))
                                .waitSeconds(0.5)
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s1.setPosition(S1OP))
                                .waitSeconds(0.4)
                                .lineToLinearHeading(new Pose2d(PX1-10, PY1-10, HEAD1))
                                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                                    ridicareSlide.setPower(0.7);
                                    ridicareSlide.setTargetPosition(375);
                                })
                                .lineToLinearHeading(new Pose2d(PX2,PY2, HEAD2))
                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                                    ridicareSlide.setPower(0.7);
                                    ridicareSlide.setTargetPosition(375);
                                    s1.setPosition(S1CL);

                                })

                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                    ridicareSlide.setPower(0.7);
                                    ridicareSlide.setTargetPosition(600);

                                })
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(135,PY3-6, HEAD3))
                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    ridicareSlide.setPower(0.8);
                                    ridicareSlide.setTargetPosition(TOP_POS);
                                    s1.setPosition(S1CL);
                                })
                                .lineToLinearHeading(new Pose2d(PX3,PY3, HEAD3))
                                .waitSeconds(0.5)

                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> ridicareSlide.setTargetPosition(TOP_POS - 150))
                                .waitSeconds(0.3)
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s1.setPosition(S1OP))
                                .waitSeconds(0.3)
                                //stack

                                //stack-cl
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    ridicareSlide.setPower(0.7);
                                    ridicareSlide.setTargetPosition(70);
                                })

                                /*.lineToLinearHeading(new Pose2d(F * 2 - 7, F - 7, 0))

                                .waitSeconds(0.3)
                                .lineToLinearHeading(new Pose2d(15, F - 7, 0))
                                .lineToLinearHeading(new Pose2d(15, 0, 0))*/
                               //.build();
                               // .lineToLinearHeading(new Pose2d(0, F, 0))
                                .lineToLinearHeading(new Pose2d(F * 2 - 30, F+3, 0))

                                .build();
                        break;
                    case 7:
                        traj = drive.trajectorySequenceBuilder(new Pose2d())
                        //traj = drive.trajectorySequenceBuilder(new Pose2d())
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    ridicareSlide.setPower(0.4);
                                    ridicareSlide.setTargetPosition(100);
                                })
                                .lineToLinearHeading(new Pose2d(8, F - 7, 0))
                                .lineToLinearHeading(new Pose2d(F * 2 - 7, F - 7, 0))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    ridicareSlide.setPower(0.7);
                                    ridicareSlide.setTargetPosition(TOP_POS);
                                    s1.setPosition(S1CL);
                                })
                                .lineToLinearHeading(new Pose2d(PX1, PY1, HEAD1))
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> ridicareSlide.setTargetPosition(TOP_POS - 150))
                                .waitSeconds(0.5)
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s1.setPosition(S1OP))
                                .waitSeconds(0.4)
                                .lineToLinearHeading(new Pose2d(PX1-10, PY1-10, HEAD1))
                                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                                    ridicareSlide.setPower(0.7);
                                    ridicareSlide.setTargetPosition(375);
                                })
                                .lineToLinearHeading(new Pose2d(PX2,PY2, HEAD2))
                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                                    ridicareSlide.setPower(0.7);
                                    ridicareSlide.setTargetPosition(375);
                                    s1.setPosition(S1CL);

                                })

                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                    ridicareSlide.setPower(0.7);
                                    ridicareSlide.setTargetPosition(600);

                                })
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(135,PY3-6, HEAD3))
                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    ridicareSlide.setPower(0.8);
                                    ridicareSlide.setTargetPosition(TOP_POS);
                                    s1.setPosition(S1CL);
                                })
                                .lineToLinearHeading(new Pose2d(PX3,PY3, HEAD3))
                                .waitSeconds(0.5)

                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> ridicareSlide.setTargetPosition(TOP_POS - 150))
                                .waitSeconds(0.3)
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s1.setPosition(S1OP))
                                .waitSeconds(0.3)
                                //stack

                                //stack-cl
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    ridicareSlide.setPower(0.7);
                                    ridicareSlide.setTargetPosition(70);
                                })

                               /* .lineToLinearHeading(new Pose2d(F * 2 - 7, F - 7, 0))

                                .waitSeconds(0.3)
                                .lineToLinearHeading(new Pose2d(15, F - 7, 0))
                                .lineToLinearHeading(new Pose2d(15, 0, 0))*/
                                //.build();
                                //.lineToLinearHeading(new Pose2d(0, F, 0))
                               // .lineToLinearHeading(new Pose2d(F * 2 - 7, F, 0))

                                .lineToLinearHeading(new Pose2d(F * 2, 0, 0)) //final

                                .build();
                        break;
                    case 8:
                        traj = drive.trajectorySequenceBuilder(new Pose2d())
                                //traj = drive.trajectorySequenceBuilder(new Pose2d())
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    ridicareSlide.setPower(0.4);
                                    ridicareSlide.setTargetPosition(100);
                                })
                                .lineToLinearHeading(new Pose2d(8, F - 7, 0))
                                .lineToLinearHeading(new Pose2d(F * 2 - 7, F - 7, 0))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    ridicareSlide.setPower(0.7);
                                    ridicareSlide.setTargetPosition(TOP_POS);
                                    s1.setPosition(S1CL);
                                })
                                .lineToLinearHeading(new Pose2d(PX1, PY1, HEAD1))
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> ridicareSlide.setTargetPosition(TOP_POS - 150))
                                .waitSeconds(0.5)
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s1.setPosition(S1OP))
                                .waitSeconds(0.4)
                                .lineToLinearHeading(new Pose2d(PX1-10, PY1-10, HEAD1))
                                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                                    ridicareSlide.setPower(0.7);
                                    ridicareSlide.setTargetPosition(375);
                                })
                                .lineToLinearHeading(new Pose2d(PX2,PY2, HEAD2))
                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                                    ridicareSlide.setPower(0.7);
                                    ridicareSlide.setTargetPosition(375);
                                    s1.setPosition(S1CL);

                                })

                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                    ridicareSlide.setPower(0.7);
                                    ridicareSlide.setTargetPosition(600);

                                })
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(135,PY3-6, HEAD3))
                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    ridicareSlide.setPower(0.8);
                                    ridicareSlide.setTargetPosition(TOP_POS);
                                    s1.setPosition(S1CL);
                                })
                                .lineToLinearHeading(new Pose2d(PX3,PY3, HEAD3))
                                .waitSeconds(0.5)

                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> ridicareSlide.setTargetPosition(TOP_POS - 150))
                                .waitSeconds(0.3)
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s1.setPosition(S1OP))
                                .waitSeconds(0.3)
                                //stack

                                //stack-cl
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    ridicareSlide.setPower(0.7);
                                    ridicareSlide.setTargetPosition(70);
                                })
                                /*.lineToLinearHeading(new Pose2d(F * 2 - 7, F - 7, 0))

                                .waitSeconds(0.3)
                                .lineToLinearHeading(new Pose2d(15, F - 7, 0))
                                .lineToLinearHeading(new Pose2d(15, 0, 0))*/
                               // .build();

                               // .lineToLinearHeading(new Pose2d(0, -F, 0))
                                .lineToLinearHeading(new Pose2d(F * 2 - 7, -F, 0))
                                .build();
                        break;
                    default:
                        traj = drive.trajectorySequenceBuilder(new Pose2d())
                                //traj = drive.trajectorySequenceBuilder(new Pose2d())
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    ridicareSlide.setPower(0.4);
                                    ridicareSlide.setTargetPosition(100);
                                })
                                .lineToLinearHeading(new Pose2d(8, F - 7, 0))
                                .lineToLinearHeading(new Pose2d(F * 2 - 7, F - 7, 0))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    ridicareSlide.setPower(0.7);
                                    ridicareSlide.setTargetPosition(TOP_POS);
                                    s1.setPosition(S1CL);
                                })
                                .lineToLinearHeading(new Pose2d(PX1, PY1, HEAD1))
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> ridicareSlide.setTargetPosition(TOP_POS - 150))
                                .waitSeconds(0.5)
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s1.setPosition(S1OP))
                                .waitSeconds(0.4)
                                .lineToLinearHeading(new Pose2d(PX1-10, PY1-10, HEAD1))
                                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                                    ridicareSlide.setPower(0.7);
                                    ridicareSlide.setTargetPosition(375);
                                })
                                .lineToLinearHeading(new Pose2d(PX2,PY2, HEAD2))
                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                                    ridicareSlide.setPower(0.7);
                                    ridicareSlide.setTargetPosition(375);
                                    s1.setPosition(S1CL);

                                })

                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                    ridicareSlide.setPower(0.7);
                                    ridicareSlide.setTargetPosition(600);

                                })
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(135,PY3-6, HEAD3))
                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    ridicareSlide.setPower(0.8);
                                    ridicareSlide.setTargetPosition(TOP_POS);
                                    s1.setPosition(S1CL);
                                })
                                .lineToLinearHeading(new Pose2d(PX3,PY3, HEAD3))
                                .waitSeconds(0.5)

                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> ridicareSlide.setTargetPosition(TOP_POS - 150))
                                .waitSeconds(0.2)
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s1.setPosition(S1OP))

                                //stack

                                //stack-cl
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    ridicareSlide.setPower(0.7);
                                    ridicareSlide.setTargetPosition(70);
                                })
                                .build();
                }
            } else {
                traj = drive.trajectorySequenceBuilder(new Pose2d())
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            ridicareSlide.setPower(0.4);
                            ridicareSlide.setTargetPosition(100);
                        })
                        .lineToLinearHeading(new Pose2d(8, F - 7, 0))
                        .lineToLinearHeading(new Pose2d(F * 2 - 7, F - 7, 0))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            ridicareSlide.setPower(0.7);
                            ridicareSlide.setTargetPosition(TOP_POS);
                            s1.setPosition(S1CL);
                        })
                        .lineToLinearHeading(new Pose2d(PX1, PY1, HEAD1))
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> ridicareSlide.setTargetPosition(TOP_POS - 150))
                        .waitSeconds(0.5)
                        .UNSTABLE_addTemporalMarkerOffset(0.1, () -> s1.setPosition(S1OP))
                        .waitSeconds(0.4)
                        .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                            ridicareSlide.setPower(0.3);
                            ridicareSlide.setTargetPosition(100);
                        })
                        .waitSeconds(0.3)
                        .lineToLinearHeading(new Pose2d(F * 2 - 7, F - 7, 0))

                        .waitSeconds(0.3)
                        .lineToLinearHeading(new Pose2d(15, F - 7, 0))
                        .lineToLinearHeading(new Pose2d(15, 0, 0))
                        .build();
                    /*
                    .splineToLinearHeading(new Pose2d(PX2, PY2, HEAD2), HEAD2)
                    .waitSeconds(2)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> s1.setPosition(S1CL))
                    .UNSTABLE_addTemporalMarkerOffset(1, () -> ridicareSlide.setTargetPosition(TOP_POS))
                    .splineToLinearHeading(new Pose2d(PX1, PY1, HEAD1), HEAD1)
                    .waitSeconds(2)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> s1.setPosition(S1OP))
                    .UNSTABLE_addTemporalMarkerOffset(0.7, () -> ridicareSlide.setTargetPosition(0))
                    .build();*/
            }

            drive.followTrajectorySequenceAsync(traj);
            drive.update();
            while (drive.isBusy() && !isStopRequested()) {
                telemetry.addData("Currently following traj", traj);
                telemetry.addData("Normal", NORMAL);
                drive.update();
            }
        /*if (NORMAL != 0) {
            if (isStopRequested()) {
                return;
            }
            Pose2d cp = drive.getPoseEstimate();
            switch (LAST_ID) {
                case 6:
                    traj = drive.trajectorySequenceBuilder(cp)
                            .splineToLinearHeading(new Pose2d(F * 2 - 7, F, 0), 0)
                            .build();
                    break;
                case 7:
                    traj = drive.trajectorySequenceBuilder(cp)
                            .splineToLinearHeading(new Pose2d(F * 2, 0, 0), 0)
                            .build();
                    break;
                case 8:
                    traj = drive.trajectorySequenceBuilder(cp)
                            .splineToLinearHeading(new Pose2d(F * 2 - 7, -F, 0), 0)
                            .build();
                    break;
                default:
                    traj = drive.trajectorySequenceBuilder(cp)
                            .waitSeconds(1)
                            .build();
            }
            drive.followTrajectorySequenceAsync(traj);
            drive.update();
            while (drive.isBusy() && !isStopRequested()) {
                telemetry.addData("Currently following second traj", traj);
                drive.update();
            }
        }*/

        }
    }
}