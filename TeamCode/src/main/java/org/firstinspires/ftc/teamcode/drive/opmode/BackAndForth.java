package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.Autonoooooooooom.HEAD1;
import static org.firstinspires.ftc.teamcode.Autonoooooooooom.HEAD2;
import static org.firstinspires.ftc.teamcode.Autonoooooooooom.HEAD3;
import static org.firstinspires.ftc.teamcode.Autonoooooooooom.HEADC;
import static org.firstinspires.ftc.teamcode.Autonoooooooooom.P1X;
import static org.firstinspires.ftc.teamcode.Autonoooooooooom.P1Y;
import static org.firstinspires.ftc.teamcode.Autonoooooooooom.P2X;
import static org.firstinspires.ftc.teamcode.Autonoooooooooom.P2Y;
import static org.firstinspires.ftc.teamcode.Autonoooooooooom.PX1;
import static org.firstinspires.ftc.teamcode.Autonoooooooooom.PX2;
import static org.firstinspires.ftc.teamcode.Autonoooooooooom.PX3;
import static org.firstinspires.ftc.teamcode.Autonoooooooooom.PXC;
import static org.firstinspires.ftc.teamcode.Autonoooooooooom.PXXC;
import static org.firstinspires.ftc.teamcode.Autonoooooooooom.PY1;
import static org.firstinspires.ftc.teamcode.Autonoooooooooom.PY2;
import static org.firstinspires.ftc.teamcode.Autonoooooooooom.PY3;
import static org.firstinspires.ftc.teamcode.Autonoooooooooom.PYC;
import static org.firstinspires.ftc.teamcode.Autonoooooooooom.PYYC;
import static org.firstinspires.ftc.teamcode.RobotConstants.SINCHIS;
import static org.firstinspires.ftc.teamcode.RobotConstants.TOP_POS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Encoder;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
//@Config
//@Disabled
@Autonomous(group = "drive")
public class BackAndForth extends LinearOpMode {

    public static double DISTANCE = 200;
    public static double HEAD = 0;

    public double H21 = -1;
    public double H22 = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        // SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        /*Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(DISTANCE)
                .build();*/

        /*Trajectory trajectoryforward = drive.trajectorybuilder(new pose2d())
                .forward(distance)
                .build();

        Trajectory trajectoryBackward = drive.trajectoryBuilder(trajectoryForward.end())
                .back(DISTANCE)
                .build();*/

        waitForStart();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //drive.setPoseEstimate(new Pose2d(PX1, PY1, HEAD1));
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        double reps = 0;
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        while (!isStopRequested()) {
            // TrajectorySequence ct = drive.trajectorySequenceBuilder(new Pose2d()).lineToLinearHeading(new Pose2d(DISTANCE, 0, HEAD)).lineToLinearHeading(new Pose2d(0, 0, 0)).build();
            if (!drive.isBusy()) {
                Vector2d P1 = new Vector2d(P1X, P1Y);
                Vector2d P2 = new Vector2d(P2X, P2Y);
                TrajectorySequence ct = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .funnyRaikuCurve(new Pose2d(PX2 - PX1 + reps * PXXC, PY2 - PY1 + reps * PYYC, HEAD2), P1, P2, H21, H22)
                        .funnyRaikuCurve(new Pose2d(PX3 - PX1 + reps * PXC, PY3 - PY1 + reps * PYC, HEAD3 + reps * HEADC), P2, P1, H21, H22)
                        .build();
                drive.followTrajectorySequenceAsync(ct);
                reps += 1;
            }
            drive.update();
            TelemetryPacket pack = new TelemetryPacket();
            pack.put("Ch", drive.getPoseEstimate().getHeading());
            pack.put("Che", drive.getLastError().getHeading());
            FtcDashboard.getInstance().sendTelemetryPacket(pack);
        }
    }
}