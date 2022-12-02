package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.List;

@Config
@Autonomous(group = "drive")
public class Autonoooooooooom extends LinearOpMode {

    private final FtcDashboard dashboard;

    public static int DIST = 0;

    public Autonoooooooooom() {
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        TrajectorySequence tf = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(DIST)
                .back(DIST)
                .build();

        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "LB");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "RB");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "RF");

        waitForStart();

        TelemetryPacket packet;

        //drive.followTrajectoryAsync(traj);

        while (!isStopRequested()) {
            drive.followTrajectorySequence(tf);
            packet = new TelemetryPacket();
            packet.put("BAT", batteryVoltageSensor.getVoltage());
            dashboard.sendTelemetryPacket(packet);
        }
        // drive.followTrajectory(trajectory);
        // drive.update();
    }
}
