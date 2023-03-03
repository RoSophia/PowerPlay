package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.S1PC;
import static org.firstinspires.ftc.teamcode.RobotConstants.S1PO;
import static org.firstinspires.ftc.teamcode.RobotConstants.S2PC;
import static org.firstinspires.ftc.teamcode.RobotConstants.S3PC;
import static org.firstinspires.ftc.teamcode.RobotConstants.SDESCHIS;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Vector;

@Config
@TeleOp(group = "drive")
@SuppressLint("DefaultLocale")
public class Test extends LinearOpMode {

    private final FtcDashboard dashboard;

    SampleMecanumDrive drive;

    public Test() {
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
    }

    double fixRetardation(double r) {
        if (r < 0) {
            return Math.PI * 2 + r;
        }
        return r;
    }

    public DcMotorEx leftBack;
    public DcMotorEx leftFront;
    public DcMotorEx rightBack;
    public DcMotorEx rightFront;
    public static boolean brak = true;

    public static double XP = 1.0;
    public static double YP = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo s1 = hardwareMap.get(Servo.class, "S1");
        DcMotor ridicareSlide = hardwareMap.get(DcMotor.class, "RS");
        ridicareSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ridicareSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        ridicareSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive = new SampleMecanumDrive(hardwareMap);

        Servo S1 = hardwareMap.get(Servo.class, "SPe");
        Servo S2 = hardwareMap.get(Servo.class, "SPa1");
        Servo S3 = hardwareMap.get(Servo.class, "SPa2");

        S1.setPosition(S1PC);
        S2.setPosition(S2PC);
        S3.setPosition(S3PC);

        Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "LED"));
        Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "Underglow"));
        Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "LB"));
        leftEncoder.setDirection(Encoder.Direction.REVERSE);

        //frontEncoder.setDirection(Encoder.Direction.REVERSE);
        //leftEncoder.setDirection(Encoder.Direction.REVERSE);

        leftBack = hardwareMap.get(DcMotorEx.class, "LB");
        leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        rightBack = hardwareMap.get(DcMotorEx.class, "RB");
        rightFront = hardwareMap.get(DcMotorEx.class, "RF");

        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);

        if (!(rightBack.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE) && brak) {
            rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        } else {
            rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        }

        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        /*BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.temperatureUnit = BNO055IMU.TempUnit.CELSIUS;
        imu.initialize(parameters);*/

        TelemetryPacket packet;
        waitForStart();
        ridicareSlide.setPower(0.5);
        ridicareSlide.setTargetPosition(40);
        ridicareSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime timer = new ElapsedTime(0);
        while (!isStopRequested()) {
            s1.setPosition(SDESCHIS);
            S1.setPosition(S1PC);
            S2.setPosition(S2PC);
            S3.setPosition(S3PC);
            if (!(rightBack.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE) && brak) {
                rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            } else if (!brak) {
                rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            }
            final double speed = Math.hypot(gamepad1.left_stick_x * XP, gamepad1.left_stick_y * YP);
            final double angle = Math.atan2(gamepad1.left_stick_y * YP, gamepad1.left_stick_x * XP) - Math.PI / 4;
            final double turn = -gamepad1.right_stick_x;
            final double ms = speed * Math.sin(angle);
            final double mc = speed * Math.cos(angle);
            //maths (nu stiu eu deastea ca fac cu antohe)
            final double lfPower = ms + turn;
            final double rfPower = mc - turn;
            final double lbPower = mc + turn;
            final double rbPower = ms - turn;

            drive.updatePoseEstimate();
            packet = new TelemetryPacket();
            packet.put("px", drive.getPoseEstimate().getX());
            packet.put("py", drive.getPoseEstimate().getY());
            packet.put("ph", drive.getPoseEstimate().getHeading());
            packet.put("El", leftEncoder.getCurrentPosition());
            packet.put("Er", rightEncoder.getCurrentPosition());
            packet.put("Ef", frontEncoder.getCurrentPosition());
            packet.put("vel", leftEncoder.getCorrectedVelocity());
            packet.put("ver", rightEncoder.getCorrectedVelocity());
            packet.put("vef", frontEncoder.getCorrectedVelocity());
            packet.put("pvel", drive.getLocalizer().getPoseVelocity());
            //packet.put("IMUH", fixRetardation(imu.getAngularOrientation().firstAngle));

            /*
            double ahe = Math.abs(fixRetardation(imu.getAngularOrientation().firstAngle) - drive.getPoseEstimate().getHeading());
            if (Math.PI * 2 - ahe < ahe) {
                ahe = Math.PI * 2 -ahe;
            }
            packet.put("Ahe", ahe);*/
            drive.update();
            telemetry.addData("px", drive.getPoseEstimate().getX());
            telemetry.addData("py", drive.getPoseEstimate().getY());
            telemetry.addData("ph", Math.toDegrees(drive.getPoseEstimate().getHeading()));
            telemetry.update();

            final double pcoef = 14.0 / batteryVoltageSensor.getVoltage();
            final double spcoef = 1 - 0.6 * gamepad1.right_trigger;
            final double fcoef = pcoef * spcoef;
            leftFront.setPower(lfPower * fcoef);
            rightFront.setPower(rfPower * fcoef);
            leftBack.setPower(lbPower * fcoef);
            rightBack.setPower(rbPower * fcoef);
            packet.put("CycleTime", timer.seconds());
            timer.reset();
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
