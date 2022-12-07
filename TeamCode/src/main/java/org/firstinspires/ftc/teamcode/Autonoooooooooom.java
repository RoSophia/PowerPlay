package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.PwrDeterminationPipeline;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.Vector;

@Config
@Autonomous(group = "drive")
public class Autonoooooooooom extends LinearOpMode {

    private final FtcDashboard dashboard;

    OpenCvCamera phoneCam;
    PwrDeterminationPipeline pipeline;

    SampleMecanumDrive drive;

    public Autonoooooooooom() {
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
    }

    public static double X1 = 50;
    public static double Y1 = 0;
    public static double H1 = Math.PI / 2;
    public static double X2 = -50;
    public static double Y2 = 0;
    public static double H2 = 0;
    public static int MODE = 1;

    Vector<Pose2d> Err = new Vector<Pose2d>();
    Vector<Long>   Tim = new Vector<Long>  ();

    long lastT = 0;

    void getErr() {
        Err.add(drive.getLastError());
        long ct = System.currentTimeMillis();
        Tim.add(ct - lastT);
        lastT = ct;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new PwrDeterminationPipeline();
        phoneCam.setPipeline(pipeline);
        //phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.MAXIMIZE_EFFICIENCY);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        //Pose2d P1 = new Pose2d(X1, Y1, H1);
        //Pose2d P2 = new Pose2d(X2, Y2, H2);
        Pose2d P1 = new Pose2d(X1, Y1, H1);
        Pose2d P2 = new Pose2d(0, 0, 0);



        TrajectorySequence t1 = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(P1)
                .addTemporalMarker(this::getErr)
                .lineToLinearHeading(P2)
                .addTemporalMarker(this::getErr)
                .build();
        lastT = System.currentTimeMillis();


        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "LB");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "RB");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "RF");

        Encoder parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "EPa"));
        //Encoder perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "EPe"));
        Encoder perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "LF"));

        waitForStart();

        TelemetryPacket packet;

        //drive.followTrajectoryAsync(traj);

        dashboard.startCameraStream(phoneCam, 15);

        while (!isStopRequested()) {
            if (MODE == 1) {
                if (!drive.isBusy()) {
                    lastT = System.currentTimeMillis();
                    drive.followTrajectorySequenceAsync(t1);
                }
                drive.update();
            } else {
                drive.updatePoseEstimate();
            }
            double xma = 0;
            double yma = 0;
            double hma = 0;
            for (int i = 0; i < Err.size(); ++i) {
                Pose2d cp = Err.get(i);
                xma = Math.max(Math.abs(cp.getX()), xma);
                yma = Math.max(Math.abs(cp.getY()), yma);
                hma = Math.max(Math.abs(cp.getHeading()), hma);
            }
            telemetry.addData("Xma", xma);
            telemetry.addData("Yma", yma);
            telemetry.addData("Hma", hma);
            telemetry.update();

            /*packet = new TelemetryPacket();
            packet.put("BAT", batteryVoltageSensor.getVoltage());
            packet.put("X", drive.getPoseEstimate().getX());
            packet.put("Y", drive.getPoseEstimate().getY());
            packet.put("H", drive.getPoseEstimate().getHeading());
            packet.put("EPa", parallelEncoder.getCurrentPosition());
            packet.put("EPe", perpendicularEncoder.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);*/
        }
    }

}
