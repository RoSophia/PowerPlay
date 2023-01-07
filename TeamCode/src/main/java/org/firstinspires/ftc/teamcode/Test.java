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
@SuppressLint("DefaultLocale")
public class Test extends LinearOpMode {

    private final FtcDashboard dashboard;

    OpenCvCamera webcam;
    AprilTagDetectionPipeline pipeline;

    SampleMecanumDrive drive;

    public Test() {
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

    double S1CL = 0.40;
    double S1OP = 0.75;

    public static double HEAD1 = Math.toRadians(311);
    public static double PX1 = 142;
    public static double PY1 = 43;

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
        waitForStart();
        ridicareSlide.setPower(0.5);
        ridicareSlide.setTargetPosition(40);
        ridicareSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        TrajectorySequence ct;
        while (!isStopRequested()) {
            drive.updatePoseEstimate();
            packet = new TelemetryPacket();
            packet.put("px", drive.getPoseEstimate().getX());
            packet.put("py", drive.getPoseEstimate().getY());
            packet.put("ph", drive.getPoseEstimate().getHeading());
            dashboard.sendTelemetryPacket(packet);
            //drive.followTrajectorySequenceAsync(traj);
            drive.update();
            telemetry.addData("px",drive.getPoseEstimate().getX());
            telemetry.addData("py",drive.getPoseEstimate().getY());
            telemetry.addData("ph",drive.getPoseEstimate().getHeading());
            telemetry.update();

        }
    }
}