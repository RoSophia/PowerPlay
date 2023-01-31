package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.SINCHIS;

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

import java.util.ArrayList;
import java.util.Vector;

@Config
@Autonomous(group = "drive")
@SuppressLint("DefaultLocale")
public class AutonoomSTANGAUwU extends LinearOpMode {

    private final FtcDashboard dashboard;

    OpenCvCamera webcam;
    AprilTagDetectionPipeline pipeline;

    SampleMecanumDrive drive;

    public AutonoomSTANGAUwU() {
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

    public static int F = 65;

    public static boolean AAAAAAAAAAAAAA = false;
    public static boolean OPENED;

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
        s1.setPosition(SINCHIS);

        drive = new SampleMecanumDrive(hardwareMap);

        //VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        TelemetryPacket packet;//

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

        packet = new TelemetryPacket();
        packet.put("LID", LAST_ID);
        dashboard.sendTelemetryPacket(packet);

        TrajectorySequence traj = null;
        switch (LAST_ID) {
            case 6:
                traj = drive.trajectorySequenceBuilder(new Pose2d())
                        .lineToLinearHeading(new Pose2d(0, F, 0))
                        .lineToLinearHeading(new Pose2d(F * 2 - 7, F, 0))
                        .build();
                break;
            default:
            case 7:
                traj = drive.trajectorySequenceBuilder(new Pose2d())
                        //.lineToLinearHeading(new Pose2d(0, F, 0))
                        .lineToLinearHeading(new Pose2d(F * 2 - 7, F, 0))
                        .lineToLinearHeading(new Pose2d(F * 2, 0, 0))
                        .build();
                break;
            case 8:
                traj = drive.trajectorySequenceBuilder(new Pose2d())
                        .lineToLinearHeading(new Pose2d(0, -F, 0))
                        .lineToLinearHeading(new Pose2d(F * 2 - 7, -F, 0))
                        .build();
                break;
        }
        follow_traj(traj);
    }
}