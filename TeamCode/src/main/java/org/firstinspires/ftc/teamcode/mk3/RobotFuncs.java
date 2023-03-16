package org.firstinspires.ftc.teamcode.mk3;

import static org.firstinspires.ftc.teamcode.RobotVars.CU_TESTING;
import static org.firstinspires.ftc.teamcode.RobotVars.DOT;
import static org.firstinspires.ftc.teamcode.RobotVars.EAP;
import static org.firstinspires.ftc.teamcode.RobotVars.EBP;
import static org.firstinspires.ftc.teamcode.RobotVars.EMAX;
import static org.firstinspires.ftc.teamcode.RobotVars.EXTT;
import static org.firstinspires.ftc.teamcode.RobotVars.IN_TESTING;
import static org.firstinspires.ftc.teamcode.RobotVars.RAP;
import static org.firstinspires.ftc.teamcode.RobotVars.RBOT_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.RETT;
import static org.firstinspires.ftc.teamcode.RobotVars.RTOP_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.SAG;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAG;
import static org.firstinspires.ftc.teamcode.RobotVars.SCO;
import static org.firstinspires.ftc.teamcode.RobotVars.SDESCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.SDIF;
import static org.firstinspires.ftc.teamcode.RobotVars.SDIP;
import static org.firstinspires.ftc.teamcode.RobotVars.SHG;
import static org.firstinspires.ftc.teamcode.RobotVars.TESTINGID;
import static org.firstinspires.ftc.teamcode.RobotVars.UPT;
import static org.firstinspires.ftc.teamcode.RobotVars.USE_PHOTON;
import static org.firstinspires.ftc.teamcode.RobotVars.coneReady;
import static org.firstinspires.ftc.teamcode.RobotVars.ebp;
import static org.firstinspires.ftc.teamcode.RobotVars.ed;
import static org.firstinspires.ftc.teamcode.RobotVars.ef;
import static org.firstinspires.ftc.teamcode.RobotVars.ei;
import static org.firstinspires.ftc.teamcode.RobotVars.ep;
import static org.firstinspires.ftc.teamcode.RobotVars.pcoef;
import static org.firstinspires.ftc.teamcode.RobotVars.rbp;
import static org.firstinspires.ftc.teamcode.RobotVars.rd;
import static org.firstinspires.ftc.teamcode.RobotVars.rf;
import static org.firstinspires.ftc.teamcode.RobotVars.ri;
import static org.firstinspires.ftc.teamcode.RobotVars.rp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@SuppressWarnings("ALL")
public class RobotFuncs {
    public static DcMotorEx ridA;
    //public static DcMotorEx ridB;
    public static DcMotorEx extA, extB;
    public static Servo sextA, sextB;
    public static Servo sClose, sHeading, sBalans, sMCLaw;
    public static PIDF epd, rpd;
    public static Clown clo;
    public static BNO055IMU imu;
    public static DcMotorEx leftBack, leftFront, rightBack, rightFront;
    public static VoltageSensor batteryVoltageSensor;
    public static FtcDashboard dashboard;
    public static HardwareMap hardwareMap;

    static void ep(double p) {
        extA.setPower(p * EAP);
        extB.setPower(p * EBP);
    }

    static void rp(double p) {
        ridA.setPower(p * RAP);
        //ridB.setPower(p * RBP);
    }

    static void spe(boolean er, double p) {
        if (!er) {
            if (p == 0) {
                epd.use = true;
            } else {
                epd.use = false;
                if (p < 0 && extA.getCurrentPosition() < -2) {
                    ep(0);
                    return;
                }
                if (p > 0 && extA.getCurrentPosition() > EMAX) {
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
                if (p > 0 && ridA.getCurrentPosition() > RTOP_POS) {
                    rp(rpd.f * 1.1);
                    return;
                }
                rp(p);
            }
        }
    }

    static void rid(int pos) {
        if (coneReady || pos == RBOT_POS || CU_TESTING) {
            if (pos == RBOT_POS) {
                sMCLaw.setPosition(SCO);
                coneReady = false;
                rpd.set_target(pos, DOT);
            } else {
                rpd.set_target(pos, UPT);
            }
        } else {
            clo.toGet = true;
        }
    }

    static public void conversiePerverssa(double p) {
        // MidFunnyRaikuStabilizer
        sextA.setPosition(p);
        sextB.setPosition(1 - p + SDIF + (1 - p) * SDIP);
    }

    static void ext(int pos) {
        TelemetryPacket cp = new TelemetryPacket();
        if (pos == EMAX) {
            clo.cext = true;
            clo.toGet = true;
            coneReady = false;
            cp.put("GoToPos", pos);
            cp.put("GoToTim", EXTT);
            epd.set_target(pos, EXTT);
        } else {
            clo.cext = false;
            cp.put("GoToPos", pos);
            cp.put("GoToTim", RETT);
            epd.set_target(pos, RETT);
        }
        dashboard.sendTelemetryPacket(cp);
        rid(RBOT_POS);
    }

    static DcMotorEx initm(String s, boolean e, boolean r) {
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

    public static void initma(HardwareMap ch) {
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
        extA = initm("extA", true, false);
        extB = initm("extB", true, true);
        ridA = initm("ridA", true, false);
        //ridB = initm("ridB", true, false);
        //underglow = hardwareMap.get(DcMotor.class, "Underglow"); You will not be forgotten
        sClose =
                sHeading =
                        sBalans =
                                sMCLaw =
                                        sextA =
                                                sextB = hardwareMap.get(Servo.class, "Toate");


        if (!IN_TESTING) {
            sClose = hardwareMap.get(Servo.class, "sClose");
            sHeading = hardwareMap.get(Servo.class, "sHeading");
            sBalans = hardwareMap.get(Servo.class, "sBalans");
            sMCLaw = hardwareMap.get(Servo.class, "sMCLaw");
            sextA = hardwareMap.get(Servo.class, "sextA");
            sextB = hardwareMap.get(Servo.class, "sextB");
        } else {
            if ((TESTINGID & 1) != 0) {
                sClose = hardwareMap.get(Servo.class, "sClose");
            } else if ((TESTINGID & 2) != 0) {
                sHeading = hardwareMap.get(Servo.class, "sHeading"); // sextA
            } else if ((TESTINGID & 4) != 0) {
                sBalans = hardwareMap.get(Servo.class, "sBalans");
            } else if ((TESTINGID & 8) != 0) {
                sMCLaw = hardwareMap.get(Servo.class, "sMCLaw"); // sextB
            } else if ((TESTINGID & 16) != 0) {
                sextA = hardwareMap.get(Servo.class, "sextA"); // sHeading
            } else if ((TESTINGID & 32) != 0) {
                sextB = hardwareMap.get(Servo.class, "sextB"); // sMCLaw
            }
        }

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        /*parameters.mode = BNO055IMU.SensorMode.COMPASS;
        parameters.gyroPowerMode = BNO055IMU.GyroPowerMode.FAST;
        parameters.gyroBandwidth = BNO055IMU.GyroBandwidth.HZ523; /// TODO ???????
        parameters.gyroRange = BNO055IMU.GyroRange.DPS2000;*/
        imu.initialize(parameters);

        epd = new PIDF(extA, extB, "Ex", ep, ed, ei, ef, ebp);
        rpd = new PIDF(ridA, null, "Ri", rp, rd, ri, rf, rbp);
        clo = new Clown(sextA, sextB, sHeading, sClose, sMCLaw, sBalans, extA);

        extT = new Thread(epd);
        ridT = new Thread(rpd);
        cloT = new Thread(clo);
    }

    public static void startma(LinearOpMode lom, boolean im) {
        pcoef = 12.0 / batteryVoltageSensor.getVoltage();

        if (im) {
            conversiePerverssa(SAG);
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

    public static void endma() {
        epd.shouldClose = true;
        rpd.shouldClose = true;
        clo.shouldClose = true;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        ridA.setPower(0);
        //ridB.setPower(0);
        extA.setPower(0);
        extB.setPower(0);
        imu.close();
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
