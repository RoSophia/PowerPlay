package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.RobotVars.FER;
import static org.firstinspires.ftc.teamcode.RobotVars.FES;
import static org.firstinspires.ftc.teamcode.RobotVars.LER;
import static org.firstinspires.ftc.teamcode.RobotVars.LES;
import static org.firstinspires.ftc.teamcode.RobotVars.RER;
import static org.firstinspires.ftc.teamcode.RobotVars.RES;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1.75;
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 11.65; // distance between the left and right wheels
    public static double FORWARD_OFFSET = 26.15; // offset of the lateral wheel

    private final Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(FORWARD_OFFSET  / 2, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(FORWARD_OFFSET / 2, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(-FORWARD_OFFSET / 2, LATERAL_DISTANCE / 2, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, LES));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, RES));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, FES));
        leftEncoder.setDirection(LER ? Encoder.Direction.REVERSE : Encoder.Direction.FORWARD);
        rightEncoder.setDirection(RER ? Encoder.Direction.REVERSE : Encoder.Direction.FORWARD);
        frontEncoder.setDirection(FER ? Encoder.Direction.REVERSE : Encoder.Direction.FORWARD);

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(frontEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()),
                encoderTicksToInches(frontEncoder.getCorrectedVelocity())
        );
    }

    @NonNull
    @Override
    public List<Double> getRunningTimes() {
        return Arrays.asList(1.0, 1.0, 1.0);
    }
}
