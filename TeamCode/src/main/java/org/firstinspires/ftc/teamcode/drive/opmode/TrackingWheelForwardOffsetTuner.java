package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.endma;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.initma;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.log_state;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.startma;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.MovingStatistics;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.mk3.RobotFuncs;

/**
 * This routine determines the effective forward offset for the lateral tracking wheel.
 * The procedure executes a point turn at a given angle for a certain number of trials,
 * along with a specified delay in milliseconds. The purpose of this is to track the
 * change in the y position during the turn. The offset, or distance, of the lateral tracking
 * wheel from the center or rotation allows the wheel to spin during a point turn, leading
 * to an incorrect measurement for the y position. This creates an arc around around
 * the center of rotation with an arc length of change in y and a radius equal to the forward
 * offset. We can compute this offset by calculating (change in y position) / (change in heading)
 * which returns the radius if the angle (change in heading) is in radians. This is based
 * on the arc length formula of length = theta * radius.
 *
 * To run this routine, simply adjust the desired angle and specify the number of trials
 * and the desired delay. Then, run the procedure. Once it finishes, it will print the
 * average of all the calculated forward offsets derived from the calculation. This calculated
 * forward offset is then added onto the current forward offset to produce an overall estimate
 * for the forward offset. You can run this procedure as many times as necessary until a
 * satisfactory result is produced.
 */
//@Config
@Disabled
@Autonomous(group="drive")
public class TrackingWheelForwardOffsetTuner extends LinearOpMode {
    public static double ANGLE = 360; // deg
    public static int NUM_TRIALS = 5;
    public static int DELAY = 300; // ms

    double fixRetardation(double r) {
        if (r < 0) {
            return Math.PI * 2 + r;
        }
        return r;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        initma(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotFuncs.drive = drive;

        if (!(drive.getLocalizer() instanceof StandardTrackingWheelLocalizer)) {
            RobotLog.setGlobalErrorMsg("StandardTrackingWheelLocalizer is not being set in the "
                    + "drive class. Ensure that \"setLocalizer(new StandardTrackingWheelLocalizer"
                    + "(hardwareMap));\" is called in SampleMecanumDrive.java");
        }

        telemetry.addLine("Press play to begin the forward offset tuner");
        telemetry.addLine("Make sure your robot has enough clearance to turn smoothly");
        telemetry.update();

        /*BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);*/

        waitForStart();

        if (isStopRequested()) return;
        startma(this, telemetry);

        telemetry.clearAll();
        telemetry.addLine("Running...");
        telemetry.update();

        MovingStatistics forwardOffsetStats = new MovingStatistics(NUM_TRIALS);
        double thead = 0;
        for (int i = 0; i < NUM_TRIALS; i++) {
            //drive.setPoseEstimate(new Pose2d());

            // it is important to handle heading wraparounds
            double headingAccumulator = 0;
            double lastHeading = 0;

            drive.turnAsync(Math.toRadians(ANGLE));

            while (!isStopRequested() && drive.isBusy()) {
                log_state();

                double heading;
                heading = drive.tl.getPoseEstimate().getHeading();
                headingAccumulator += Angle.norm(heading - lastHeading);
                lastHeading = heading;

                drive.update();
            }

            double forwardOffset = StandardTrackingWheelLocalizer.FORWARD_OFFSET +
                    drive.tl.getPoseEstimate().getY() / headingAccumulator;
            forwardOffsetStats.add(forwardOffset);
            thead += headingAccumulator;

            sleep(DELAY);
        }

        telemetry.clearAll();
        telemetry.addLine("Tuning complete");
        telemetry.addData("Total heading", thead);
        telemetry.addLine(Misc.formatInvariant("Effective forward offset = %.2f (SE = %.3f)",
                forwardOffsetStats.getMean(),
                forwardOffsetStats.getStandardDeviation() / Math.sqrt(NUM_TRIALS)));
        telemetry.update();

        endma();
        while (!isStopRequested()) {
            idle();
        }
    }
}
