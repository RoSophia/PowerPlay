package org.firstinspires.ftc.teamcode.mk3;

import static org.firstinspires.ftc.teamcode.RobotVars.AUTO_CLOW;
import static org.firstinspires.ftc.teamcode.RobotVars.CU_TESTING;
import static org.firstinspires.ftc.teamcode.RobotVars.DOT;
import static org.firstinspires.ftc.teamcode.RobotVars.EAP;
import static org.firstinspires.ftc.teamcode.RobotVars.EBP;
import static org.firstinspires.ftc.teamcode.RobotVars.EMAX;
import static org.firstinspires.ftc.teamcode.RobotVars.EMIN;
import static org.firstinspires.ftc.teamcode.RobotVars.EXTT;
import static org.firstinspires.ftc.teamcode.RobotVars.FER;
import static org.firstinspires.ftc.teamcode.RobotVars.FES;
import static org.firstinspires.ftc.teamcode.RobotVars.LEEW;
import static org.firstinspires.ftc.teamcode.RobotVars.LER;
import static org.firstinspires.ftc.teamcode.RobotVars.LES;
import static org.firstinspires.ftc.teamcode.RobotVars.MAX_DIF_EXT;
import static org.firstinspires.ftc.teamcode.RobotVars.MAX_DIF_RID;
import static org.firstinspires.ftc.teamcode.RobotVars.RAP;
import static org.firstinspires.ftc.teamcode.RobotVars.RBOT_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.RBP;
import static org.firstinspires.ftc.teamcode.RobotVars.RER;
import static org.firstinspires.ftc.teamcode.RobotVars.RES;
import static org.firstinspires.ftc.teamcode.RobotVars.RETT;
import static org.firstinspires.ftc.teamcode.RobotVars.RTOP_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.SAG;
import static org.firstinspires.ftc.teamcode.RobotVars.SAP;
import static org.firstinspires.ftc.teamcode.RobotVars.SAW;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAG;
import static org.firstinspires.ftc.teamcode.RobotVars.SCC;
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
import static org.firstinspires.ftc.teamcode.RobotVars.useRid;

import static java.lang.Thread.sleep;

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
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

import dalvik.system.DelegateLastClassLoader;

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
    public static LinearOpMode lom;
    public static Telemetry telemetry;

    static double eps = 0.01;

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
            if (Math.abs(p) < 0.0001) { /// If we do not manually power the motor let their PID keep them there
                epd.use = true;
            } else {
                epd.use = false; /// Turn off the PID so we can power tham manually
                if (p < 0 && extA.getCurrentPosition() < 12) { /// Prevent the extension mechanism from going beyond its bounds
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
            if (ridA == null) {
                return;
            }
            if (Math.abs(p) < 0.0001) {
                rpd.use = true;
            } else {
                rpd.use = false;
                if (p < 0 && ridA.getCurrentPosition() < 12) {
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
                if (pos == RTOP_POS) {
                    rpd.set_target(pos, UPT);
                } else {
                    rpd.set_target(pos, UPT / 2);
                }
            }
        } else {
            if (!coneClaw) {
                clo.toGet = true;
            }
        }
    }

    public static enum WAITS {TRANSFER, HOISTER, EXTENSION, HOISTER_FALL}

    public static SampleMecanumDrive drive;
    public static Encoder leftEncoder;
    public static Encoder rightEncoder;
    public static Encoder frontEncoder;

    public static void log_state() {
        TelemetryPacket pack = new TelemetryPacket();
        if (drive != null) {
            pack.put("Ex", drive.getLastError().getX());
            pack.put("Ey", drive.getLastError().getY());
            pack.put("Eh", drive.getLastError().getHeading());
        }
        pack.put("vel", leftEncoder.getCorrectedVelocity());
        pack.put("ver", rightEncoder.getCorrectedVelocity());
        pack.put("vef", frontEncoder.getCorrectedVelocity());
        pack.put("voltage", batteryVoltageSensor.getVoltage());
        if (extA != null) {
            pack.put("CUR_extA", extA.getCurrent(CurrentUnit.MILLIAMPS));
            pack.put("CUR_extB", extB.getCurrent(CurrentUnit.MILLIAMPS));
            pack.put("POW_extA", extA.getPower());
            pack.put("POW_extB", extB.getPower());
        }
        if (ridA != null) {
            pack.put("CUR_ridA", ridA.getCurrent(CurrentUnit.MILLIAMPS));
            pack.put("CUR_ridB", ridB.getCurrent(CurrentUnit.MILLIAMPS));
            pack.put("POW_ridA", ridA.getPower());
            pack.put("POW_ridB", ridB.getPower());
        }
        dashboard.sendTelemetryPacket(pack);
    }


    static double TIMEOUT = 1.6;
    public static void wtfor(WAITS p, double extra) {
        TelemetryPacket pa = new TelemetryPacket();
        pa.put("WAIT_FOR_P", p);
        pa.put("WAIT_FOR_E", extra);
        dashboard.sendTelemetryPacket(pa);
        ElapsedTime timeout = new ElapsedTime(0);
        timeout.reset();
        try {
            if (p == WAITS.TRANSFER) { /// 0: Wait for transfer to finish
                while (!lom.isStopRequested() && !coneReady && timeout.seconds() < TIMEOUT) {
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    drive.updatePoseEstimate();
                    log_state();
                    pa = new TelemetryPacket();
                    pa.put("WAIT_FOR_T", !coneReady);
                    dashboard.sendTelemetryPacket(pa);
                    /*
                    telemetry.addLine("WRFOR TRANSFER");
                    telemetry.update();

                     */
                    sleep(2);
                }
            } else if (p == WAITS.HOISTER) { /// 1: Wait for hoister to the thing
                while (!lom.isStopRequested() && (RTOP_POS - ridA.getCurrentPosition()) > MAX_DIF_RID && timeout.seconds() < TIMEOUT) {
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    drive.updatePoseEstimate();
                    log_state();
                    pa = new TelemetryPacket();
                    pa.put("WAIT_FOR_T", RTOP_POS - ridA.getCurrentPosition());
                    dashboard.sendTelemetryPacket(pa);
                    /*
                    telemetry.addLine("WRFOR HOISTER");
                    telemetry.addData("Hoister", RTOP_POS - ridA.getCurrentPosition());
                    telemetry.update();

                     */
                    sleep(2);
                }
            } else if (p == WAITS.EXTENSION) { /// 2: Wait for extension to finish
                while (!lom.isStopRequested() && (EMAX - extA.getCurrentPosition()) > MAX_DIF_EXT && timeout.seconds() < TIMEOUT) {
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    drive.updatePoseEstimate();
                    log_state();
                    pa = new TelemetryPacket();
                    pa.put("WAIT_FOR_T", EMAX - extA.getCurrentPosition());
                    dashboard.sendTelemetryPacket(pa);
                    /*
                    telemetry.addLine("WRFOR EXTENSION");
                    telemetry.addData("Extnesion", EMAX - extA.getCurrentPosition());
                    telemetry.update();

                     */
                    sleep(2);
                }
            } else if (p == WAITS.HOISTER_FALL) { /// 2: Wait for hoisster to not the thing
                while (!lom.isStopRequested() && (ridA.getCurrentPosition() - RBOT_POS) > MAX_DIF_RID && timeout.seconds() < TIMEOUT) {
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    drive.updatePoseEstimate();
                    log_state();
                    pa = new TelemetryPacket();
                    /*
                    pa.put("WAIT_FOR_T", ridA.getCurrentPosition() - RBOT_POS);
                    dashboard.sendTelemetryPacket(pa);
                    telemetry.addLine("WRFOR HOISTER_FALL");
                    telemetry.update();
                     */
                    sleep(2);
                }
            }
            ElapsedTime et = new ElapsedTime(0);
            et.reset();
            pa = new TelemetryPacket();
            pa.put("WAIT_FOR_WAIT", et.seconds());
            dashboard.sendTelemetryPacket(pa);
            while (et.seconds() < extra && !lom.isStopRequested()) {
                leftBack.setPower(0);
                rightBack.setPower(0);
                leftFront.setPower(0);
                rightFront.setPower(0);
                drive.updatePoseEstimate();
                log_state();
                pa = new TelemetryPacket();
                pa.put("WAIT_FOR_WAIT", et.seconds());
                dashboard.sendTelemetryPacket(pa);
                /*
                telemetry.addLine("WRFOR EXTRA");
                telemetry.addData("Time", et.seconds());
                telemetry.update();
                 */
                sleep(2);
            }
        } catch (Exception e) {
            leftBack.setPower(0);
            rightBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA", "KKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKkkk");
            packet.put("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAB", e.getStackTrace().toString());
            packet.put("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAC", e.toString());
            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("Wtfor Error", e.toString());
            telemetry.update();
        }
    }

    public static boolean wtfor_nonblocking(WAITS p) {
        if (p == WAITS.TRANSFER) { /// 0: Wait for transfer to finish
            return coneReady;
        } else if (p == WAITS.HOISTER) { /// 1: Wait for hoister to the thing
            return (RTOP_POS - ridA.getCurrentPosition()) < MAX_DIF_RID;
        } else if (p == WAITS.EXTENSION) { /// 2: Wait for extension to finish
            return (EMAX - extA.getCurrentPosition()) < MAX_DIF_EXT;
        }
        return false;
    }


    static public void conversiePerverssa(double p) { /// Handle moving both grabber arm servos
        // MidFunnyRaikuStabilizer
        sextA.setPosition(p);
        sextB.setPosition(p + SDIF + (1 - p) * SDIP);
    }

    static void ext(int pos) { /// Extend to a set position
        if (pos == EMAX) {
            clo.toGet = true;
            coneReady = false;
            epd.set_target(pos, EXTT);
        } else {
            epd.set_target(pos, RETT);
        }
        rid(RBOT_POS);
    }

    static DcMotorEx initm(String s, boolean e, boolean r) { /// Init a motor
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, s);

        MotorConfigurationType mconf = m.getMotorType().clone();
        mconf.setAchieveableMaxRPMFraction(1.0);
        m.setMotorType(mconf);

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

    public static void initma(HardwareMap ch, boolean AUTONOMUS) { /// Init all hardware info
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
        if (useRid) {
            ridA = initm("ridA", true, true);
            ridB = initm("ridB", true, false);
        } else {
            ridA = null;
            ridB = null;
        }
        //underglow = hardwareMap.get(DcMotor.class, "Underglow"); You will not be forgotten

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

        if (AUTONOMUS) {
            sMCLaw.setPosition(SCC);
            conversiePerverssa(SAP);
            sClose.setPosition(SDESCHIS);
            sBalans.setPosition(SBAG);
            sHeading.setPosition(SHG);
        }

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, LES));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, RES));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, FES));
        leftEncoder.setDirection(LER ? Encoder.Direction.REVERSE : Encoder.Direction.FORWARD);
        rightEncoder.setDirection(RER ? Encoder.Direction.REVERSE : Encoder.Direction.FORWARD);
        frontEncoder.setDirection(FER ? Encoder.Direction.REVERSE : Encoder.Direction.FORWARD);

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

        /*
        TelemetryPacket pack = new TelemetryPacket();
        pack.put("ID_rightFront", rightFront.getPortNumber());
        pack.put("ID_rightFrontCinfo", rightFront.getConnectionInfo());
        pack.put("ID_leftFront", leftFront.getPortNumber());
        pack.put("ID_leftFrontCinfo", leftFront.getConnectionInfo());
        pack.put("ID_rightBack", rightBack.getPortNumber());
        pack.put("ID_rightBackCinfo", rightBack.getConnectionInfo());
        pack.put("ID_leftBack", leftBack.getPortNumber());
        pack.put("ID_leftBackCinfo", leftBack.getConnectionInfo());
        pack.put("ID_ridA", ridA.getPortNumber());
        pack.put("ID_ridACinfo", ridA.getConnectionInfo());
        pack.put("ID_ridB", ridB.getPortNumber());
        pack.put("ID_ridBCinfo", ridB.getConnectionInfo());
        pack.put("ID_extA", extA.getPortNumber());
        pack.put("ID_extACinfo", extA.getConnectionInfo());
        pack.put("ID_extB", extB.getPortNumber());
        pack.put("ID_extBCinfo", extB.getConnectionInfo());
        pack.put("ID_sClose", sClose.getPortNumber());
        pack.put("ID_sCloseCinfo", sClose.getConnectionInfo());
        pack.put("ID_sHeading", sHeading.getPortNumber());
        pack.put("ID_sHeadingCinfo", sHeading.getConnectionInfo());
        pack.put("ID_sBalans", sBalans.getPortNumber());
        pack.put("ID_sBalansCinfo", sBalans.getConnectionInfo());
        pack.put("ID_sMCLaw", sMCLaw.getPortNumber());
        pack.put("ID_sMCLawCinfo", sMCLaw.getConnectionInfo());
        pack.put("ID_sextA", sextA.getPortNumber());
        pack.put("ID_sextACinfo", sextA.getConnectionInfo());
        pack.put("ID_sextB", sextB.getPortNumber());
        pack.put("ID_sextBCinfo", sextB.getConnectionInfo());
        dashboard.sendTelemetryPacket(pack);
         */


        epd = new PIDF(extA, extB, "Ex", ep, ed, ei, ef, ebp, emd);
        rpd = new PIDF(ridA, ridB, "Ri", rp, rd, ri, rf, rbp, rmd);
        clo = new Clown(sextA, sextB, sHeading, sClose, sMCLaw, sBalans, extA, sensorRange);

        extT = new Thread(epd);
        ridT = new Thread(rpd);
        cloT = new Thread(clo);
    }

    public static void startma(LinearOpMode lom, Telemetry tele, boolean im) { /// Set all values to their starting ones and start the PID threads
        pcoef = 12.0 / batteryVoltageSensor.getVoltage();
        RobotFuncs.lom = lom;
        RobotFuncs.telemetry = tele;

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
        epd.target = EMIN;
        epd.lom = lom;
        extT.start();

        rpd.shouldClose = false;
        rpd.use = true;
        rpd.target = RBOT_POS;
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
        if (ridA != null) {
            ridA.setPower(0);
            ridB.setPower(0);
        }
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
