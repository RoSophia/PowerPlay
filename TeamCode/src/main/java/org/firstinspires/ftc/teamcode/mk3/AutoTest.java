package org.firstinspires.ftc.teamcode.mk3;

import static org.firstinspires.ftc.teamcode.RobotVars.SCC;
import static org.firstinspires.ftc.teamcode.RobotVars.SHITTY_WORKAROUND_POWER;
import static org.firstinspires.ftc.teamcode.RobotVars.SHITTY_WORKAROUND_TIME;
import static org.firstinspires.ftc.teamcode.RobotVars.SINCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.coneReady;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.dashboard;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.endma;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.epd;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.extA;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.extB;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.initma;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.leftEncoder;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.log_state;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.preinit;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sClose;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sMCLaw;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.startma;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

@Config
@Autonomous(group = "drive")
@SuppressLint("DefaultLocale")
public class AutoTest extends LinearOpMode {
    SampleMecanumDrive drive;
    public static double P1H = 5.585;
    public static double P1X = -134;
    public static double P1Y = 6;

    public static double MVEL = 60;
    public static double MAL = 40;

    void set_wait_time(double t) {
        drive.follower = new HolonomicPIDVAFollower(SampleMecanumDrive.AXIAL_PID, SampleMecanumDrive.LATERAL_PID, SampleMecanumDrive.HEADING_PID, new Pose2d(2, 2, Math.toRadians(2)), t);
        drive.trajectorySequenceRunner = new TrajectorySequenceRunner(drive.follower, SampleMecanumDrive.HEADING_PID);
    }

    void follow_traj(TrajectorySequence traj) {
        if (traj == null) {
            return;
        }
        drive.followTrajectorySequenceAsync(traj);
        ElapsedTime timer = new ElapsedTime(0);
        ElapsedTime FULL_TIMER = new ElapsedTime(0);
        FULL_TIMER.reset();
        double MAX_DURATION = traj.duration();
        TelemetryPacket telepack = new TelemetryPacket();
        telepack.put("TEMP:MAX_DUR", MAX_DURATION);
        telepack.put("TEMP:FULL_TIMER", FULL_TIMER.seconds());
        dashboard.sendTelemetryPacket(telepack);
        while (drive.isBusy() && !isStopRequested() && !gamepad1.right_bumper && FULL_TIMER.seconds() <= MAX_DURATION + 0.01) {
            drive.update();
            telemetry.update();
            leftEncoder.getCorrectedVelocity();
            log_state();
            TelemetryPacket pack = new TelemetryPacket();
            pack.put("CycleTime", timer.milliseconds());
            pack.put("TargH", traj.end().getHeading());
            pack.put("TargX", traj.end().getX());
            pack.put("TargY", traj.end().getY());
            timer.reset();
            dashboard.sendTelemetryPacket(pack);

            telepack = new TelemetryPacket();
            telepack.put("TEMP:MAX_DUR", MAX_DURATION);
            telepack.put("TEMP:FULL_TIMER", FULL_TIMER.seconds());
            dashboard.sendTelemetryPacket(telepack);

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
        if (!isStopRequested()) {
            drive.update();
        }
        drive.tl.updHeading();
    }

    ElapsedTime SHITTY_WORKAROUND_TIMER = new ElapsedTime(0);
    boolean SHITTY_WORKAROUND_TIMED = false;

    void init_auto() {
        initma(hardwareMap);
        sMCLaw.setPosition(SCC);
        drive = new SampleMecanumDrive(hardwareMap);
        RobotFuncs.drive = drive;
        coneReady = true;
    }

    public static boolean SPEED = true;

    @Override
    public void runOpMode() throws InterruptedException {
        preinit();
        init_auto();

        if (isStopRequested()) {
            endma();
            return;
        }

        TelemetryPacket tttp = new TelemetryPacket();
        tttp.addLine("INIT DONE!");
        telemetry.addLine("INIT DONE!");
        telemetry.update();
        waitForStart();

        startma(this, telemetry);
        sMCLaw.setPosition(SCC);
        sClose.setPosition(SINCHIS);

        TrajectorySequence ts;
        if (!SPEED) {
            ts = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                    .lineToLinearHeading(new Pose2d(P1X, P1Y, P1H))
                    .build();
        } else {
            TrajectoryVelocityConstraint vc = SampleMecanumDrive.getVelocityConstraint(MVEL, MAX_ANG_VEL, TRACK_WIDTH);
            TrajectoryAccelerationConstraint ac = SampleMecanumDrive.getAccelerationConstraint(MAL);
            ts = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                    .lineToLinearHeading(new Pose2d(P1X, P1Y, P1H), vc, ac)
                    .build();
        }

        follow_traj(ts);

        endma();


    }
}
