package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.S1PO;
import static org.firstinspires.ftc.teamcode.RobotConstants.S2PO;
import static org.firstinspires.ftc.teamcode.RobotConstants.S3PO;
import static org.firstinspires.ftc.teamcode.RobotConstants.SDESCHIS;
import static org.firstinspires.ftc.teamcode.RobotConstants.TOP_POS;
import static org.firstinspires.ftc.teamcode.RobotConstants.USE_PHOTON;
import static org.firstinspires.ftc.teamcode.RobotConstants.pcoef;
import static org.firstinspires.ftc.teamcode.RobotConstants.rd;
import static org.firstinspires.ftc.teamcode.RobotConstants.rf;
import static org.firstinspires.ftc.teamcode.RobotConstants.ri;
import static org.firstinspires.ftc.teamcode.RobotConstants.rp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class RobotFuncs {
    public static DcMotorEx leftBack, leftFront, rightBack, rightFront;
    public static DcMotorEx ridicareSlide;
    public static DcMotor underglow;
    public static Servo s1, s2;
    public static Servo S1, S2, S3;
    //public static Rev2mDistanceSe0sor cs;
    public static VoltageSensor batteryVoltageSensor;
    public static FtcDashboard dashboard;
    public static BNO055IMU imu;
    public static PIDF armRun;
    static HardwareMap hardwareMap;
    static Thread armThread;

    static void rid(double p) {
        TelemetryPacket pa = new TelemetryPacket();
        pa.put("RidPow", p);
        dashboard.sendTelemetryPacket(pa);
        if (Math.abs(p) > 0.001) {
            armRun.use = false;
            if (ridicareSlide.getCurrentPosition() < 5 && p < 0) {
                ridicareSlide.setPower(0);
                return;
            }
            if (ridicareSlide.getCurrentPosition() > TOP_POS && p > 0) {
                ridicareSlide.setPower(0);
                return;
            }
            ridicareSlide.setPower(p);
            armRun.set_target(ridicareSlide.getCurrentPosition(), 0);
        } else {
            armRun.use = true;
        }
    }

    static DcMotorEx initm(String name, boolean enc, boolean rev) {
        DcMotorEx cm = hardwareMap.get(DcMotorEx.class, name);
        if (enc) {
            cm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            cm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            cm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        cm.setDirection(rev ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        cm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        return cm;
    }


    public static void initma(HardwareMap hwm) {
        if (USE_PHOTON) {
            PhotonCore.enable();
            PhotonCore.experimental.setSinglethreadedOptimized(false);
        }
        hardwareMap = hwm;

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        dashboard = FtcDashboard.getInstance();

        leftBack = initm("LB", false, false);
        leftFront = initm("LF", false, false);
        rightBack = initm("RB", false, true);
        rightFront = initm("RF", false, true);
        ridicareSlide = initm("RS", true, true);
        underglow = initm("Underglow", false, false);

        s1 = hardwareMap.get(Servo.class, "S1");
        S1 = hardwareMap.get(Servo.class, "SPe");
        S2 = hardwareMap.get(Servo.class, "SPa1");
        S3 = hardwareMap.get(Servo.class, "SPa2");
        //cs = hardwareMap.get(Rev2mDistanceSensor.class, "Claw");

        armRun = new PIDF(ridicareSlide, "Ri", rp, ri, rd, rf);
        armThread = new Thread(armRun);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        s1.setPosition(SDESCHIS);
        S1.setPosition(S1PO);
        S2.setPosition(S2PO);
        S3.setPosition(S3PO);
    }

    public static void startma(LinearOpMode lom) {
        pcoef = 12.0 / batteryVoltageSensor.getVoltage();

        armRun.shouldClose = false;
        armRun.use = true;
        armRun.lom = lom;
        ridicareSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ridicareSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armThread.start();
    }

    public static void endma() {
        pcoef = 0;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        try {
            armRun.shouldClose = true;
            armThread.join();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
