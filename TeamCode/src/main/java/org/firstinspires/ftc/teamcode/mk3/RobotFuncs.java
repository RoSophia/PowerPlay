package org.firstinspires.ftc.teamcode.mk3;

import static org.firstinspires.ftc.teamcode.RobotVars.AUTO_CLOW;
import static org.firstinspires.ftc.teamcode.RobotVars.CU_TESTING;
import static org.firstinspires.ftc.teamcode.RobotVars.DOT;
import static org.firstinspires.ftc.teamcode.RobotVars.EAP;
import static org.firstinspires.ftc.teamcode.RobotVars.EBP;
import static org.firstinspires.ftc.teamcode.RobotVars.EMAX;
import static org.firstinspires.ftc.teamcode.RobotVars.EXTT;
import static org.firstinspires.ftc.teamcode.RobotVars.LEEW;
import static org.firstinspires.ftc.teamcode.RobotVars.RAP;
import static org.firstinspires.ftc.teamcode.RobotVars.RBOT_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.RBP;
import static org.firstinspires.ftc.teamcode.RobotVars.RETT;
import static org.firstinspires.ftc.teamcode.RobotVars.RTOP_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.SAG;
import static org.firstinspires.ftc.teamcode.RobotVars.SAW;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAG;
import static org.firstinspires.ftc.teamcode.RobotVars.SCO;
import static org.firstinspires.ftc.teamcode.RobotVars.SDESCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.SDIF;
import static org.firstinspires.ftc.teamcode.RobotVars.SDIP;
import static org.firstinspires.ftc.teamcode.RobotVars.SHG;
import static org.firstinspires.ftc.teamcode.RobotVars.STARTW;
import static org.firstinspires.ftc.teamcode.RobotVars.UPT;
import static org.firstinspires.ftc.teamcode.RobotVars.USE_PHOTON;
import static org.firstinspires.ftc.teamcode.RobotVars.coneClaw;
import static org.firstinspires.ftc.teamcode.RobotVars.coneReady;
import static org.firstinspires.ftc.teamcode.RobotVars.ebp;
import static org.firstinspires.ftc.teamcode.RobotVars.ed;
import static org.firstinspires.ftc.teamcode.RobotVars.ef;
import static org.firstinspires.ftc.teamcode.RobotVars.ei;
import static org.firstinspires.ftc.teamcode.RobotVars.emd;
import static org.firstinspires.ftc.teamcode.RobotVars.ep;
import static org.firstinspires.ftc.teamcode.RobotVars.pcoef;
import static org.firstinspires.ftc.teamcode.RobotVars.rbp;
import static org.firstinspires.ftc.teamcode.RobotVars.rd;
import static org.firstinspires.ftc.teamcode.RobotVars.rf;
import static org.firstinspires.ftc.teamcode.RobotVars.ri;
import static org.firstinspires.ftc.teamcode.RobotVars.rmd;
import static org.firstinspires.ftc.teamcode.RobotVars.rp;
import static org.firstinspires.ftc.teamcode.RobotVars.useExt;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@SuppressWarnings("ALL")
public class RobotFuncs {
    public static DcMotorEx ridA;
    public static DcMotorEx ridB;
    public static DcMotorEx extA, extB;
    public static ServoImplEx sextA, sextB;
    public static Servo sClose, sHeading, sBalans, sMCLaw;
    public static PIDF epd, rpd;
    public static Clown clo;
    public static BNO055IMU imu;
    public static DcMotorEx leftBack, leftFront, rightBack, rightFront;
    public static VoltageSensor batteryVoltageSensor;
    public static FtcDashboard dashboard;
    public static HardwareMap hardwareMap;
    public static DistanceSensor sensorRange;

    static double eps = 0.00001;

    static boolean epsEq(double o1, double o2) {
        if (Math.abs(o1 - o2) < eps) {
            return true;
        }
        return false;
    }

    static void ep(double p) { /// Set power in the extension motors
        if (extA == null) {
            return;
        }
        extA.setPower(p * EAP);
        extB.setPower(p * EBP);
    }

    static void rp(double p) { /// Set power in the lift motors
        ridA.setPower(p * RAP);
        ridB.setPower(p * RBP);
    }

    static void spe(boolean er, double p) { /// Set power in motors (used for both sets of motors)
        /// power the extension motors if `er` is 0, otherwise power the lift ones
        if (!er) {
            if (extA == null) {
                return;
            }
            if (p == 0) { /// If we do not manually power the motor let their PID keep them there
                epd.use = true;
            } else {
                epd.use = false; /// Turn off the PID so we can power tham manually
                if (p < 0 && extA.getCurrentPosition() < -2) { /// Prevent the extension mechanism from going beyond its bounds
                    ep(0);
                    return;
                }
                if (p > 0 && extA.getCurrentPosition() > EMAX) { /// Prevent the extension mechanism from going beyond its bounds
                    ep(0);
                    return;
                }
                ep(p);
            }
        } else {
            if (p == 0) {
                rpd.use = true;
            } else {
                rpd.use = false;
                if (p < 0 && ridA.getCurrentPosition() < -2) {
                    rp(0);
                    return;
                }
                if (p > 0 && ridA.getCurrentPosition() > RTOP_POS + LEEW) {
                    rp(rpd.f * 1.1);
                    return;
                }
                rp(p);
            }
        }
    }

    static void rid(int pos) { /// Move the lift to a set position
        if (coneReady || pos == RBOT_POS || (CU_TESTING > 0)) {
            if (pos == RBOT_POS) { /// Retract the lift (lets the cone that was being held fall)
                sMCLaw.setPosition(SCO);
                coneReady = false;
                rpd.set_target(pos, DOT);
            } else {
                rpd.set_target(pos, UPT);
            }
        } else {
            if (!coneClaw) {
                clo.toGet = true;
            }
        }
    }

    static public void conversiePerverssa(double p) { /// Handle moving both grabber arm servos
        // MidFunnyRaikuStabilizer
        sextA.setPosition(p);
        sextB.setPosition(p + SDIF + (1 - p) * SDIP);
    }

    static void ext(int pos) { /// Extend to a set position
        TelemetryPacket cp = new TelemetryPacket();
        if (pos == EMAX) {
            clo.toGet = true;
            coneReady = false;
            cp.put("GoToPos", pos);
            cp.put("GoToTim", EXTT);
            epd.set_target(pos, EXTT);
        } else {
            cp.put("GoToPos", pos);
            cp.put("GoToTim", RETT);
            epd.set_target(pos, RETT);
        }
        dashboard.sendTelemetryPacket(cp);
        rid(RBOT_POS);
    }

    static DcMotorEx initm(String s, boolean e, boolean r) { /// Init a motor
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, s);
        if (e) {
            m.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        } else {
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m.setDirection(r ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        return m;
    }

    static Thread extT, ridT, cloT;

    public static void initma(HardwareMap ch) { /// Init all hardware info
        if (USE_PHOTON) {
            PhotonCore.enable();
            PhotonCore.experimental.setSinglethreadedOptimized(false);
        }
        hardwareMap = ch;

        dashboard = FtcDashboard.getInstance();
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        rightBack = initm("RB", false, true);   // P4
        rightFront = initm("RF", false, true);  // P2
        leftBack = initm("LB", false, false);   // P3
        leftFront = initm("LF", false, false);  // P1
        if (useExt) {
            extA = initm("extA", true, false);
            extB = initm("extB", true, true);
        } else {
            extA = null;
            extB = null;
        }
        ridA = initm("ridA", true, false);
        ridB = initm("ridB", false, true);
        //underglow = hardwareMap.get(DcMotor.class, "Underglow"); You will not be forgotten
        sClose = sHeading = sBalans = sMCLaw = hardwareMap.get(Servo.class, "Toate");
        sextA = sextB = (ServoImplEx) sClose;

        sClose = hardwareMap.get(Servo.class, "sClose");
        sHeading = hardwareMap.get(Servo.class, "sHeading");
        sBalans = hardwareMap.get(Servo.class, "sBalans");
        sMCLaw = hardwareMap.get(Servo.class, "sMCLaw");
        sextA = hardwareMap.get(ServoImplEx.class, "sextA");
        sextB = hardwareMap.get(ServoImplEx.class, "sextB");

        sextA.setPwmEnable();
        sextB.setPwmEnable();
        sextA.setPwmRange(new PwmControl.PwmRange(505, 2495));
        sextB.setPwmRange(new PwmControl.PwmRange(505, 2495));

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        /*parameters.mode = BNO055IMU.SensorMode.COMPASS;
        parameters.gyroPowerMode = BNO055IMU.GyroPowerMode.FAST;
        parameters.gyroBandwidth = BNO055IMU.GyroBandwidth.HZ523; /// TODO ???????
        parameters.gyroRange = BNO055IMU.GyroRange.DPS2000;*/
        imu.initialize(parameters);
        if (AUTO_CLOW) {
            sensorRange = hardwareMap.get(DistanceSensor.class, "csensor");
        } else {
            sensorRange = null;
        }

        epd = new PIDF(extA, extB, "Ex", ep, ed, ei, ef, ebp, emd);
        rpd = new PIDF(ridA, ridB, "Ri", rp, rd, ri, rf, rbp, rmd);
        clo = new Clown(sextA, sextB, sHeading, sClose, sMCLaw, sBalans, extA, sensorRange);

        extT = new Thread(epd);
        ridT = new Thread(rpd);
        cloT = new Thread(clo);
    }

    public static void startma(LinearOpMode lom, boolean im) { /// Set all values to their starting ones and start the PID threads
        pcoef = 12.0 / batteryVoltageSensor.getVoltage();

        if (im) { /// Set these positions only if called by a teleop class
            if (STARTW) {
                conversiePerverssa(SAW);
            } else {
                conversiePerverssa(SAG);
            }
            sHeading.setPosition(SHG);
            sClose.setPosition(SDESCHIS);
            sMCLaw.setPosition(SCO);
            sBalans.setPosition(SBAG);
        }

        epd.shouldClose = false;
        epd.use = true;
        epd.target = 0;
        epd.lom = lom;
        extT.start();

        rpd.shouldClose = false;
        rpd.use = true;
        rpd.target = 0;
        rpd.lom = lom;
        ridT.start();

        clo.shouldClose = false;
        cloT.start();
    }

    public static void endma() { /// Shut down the robot
        pcoef = 0;
        epd.shouldClose = true;
        rpd.shouldClose = true;
        clo.shouldClose = true;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        ridA.setPower(0);
        ridB.setPower(0);
        if (extA != null) {
            extA.setPower(0);
            extB.setPower(0);
        }
        imu.close();
        if (sensorRange != null) {
            sensorRange.close();
        }
        batteryVoltageSensor.close();
        try {
            extT.join();
            ridT.join();
            cloT.join();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
