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
import static org.firstinspires.ftc.teamcode.RobotVars.RIDICARE_LAMPREY;
import static org.firstinspires.ftc.teamcode.RobotVars.RMID_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.RTOP_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.SAW;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAD;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAG;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAS;
import static org.firstinspires.ftc.teamcode.RobotVars.SCO;
import static org.firstinspires.ftc.teamcode.RobotVars.SDIF;
import static org.firstinspires.ftc.teamcode.RobotVars.SDIP;
import static org.firstinspires.ftc.teamcode.RobotVars.SGD;
import static org.firstinspires.ftc.teamcode.RobotVars.SGS;
import static org.firstinspires.ftc.teamcode.RobotVars.SHG;
import static org.firstinspires.ftc.teamcode.RobotVars.SINCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.UPT;
import static org.firstinspires.ftc.teamcode.RobotVars.USE_PHOTON;
import static org.firstinspires.ftc.teamcode.RobotVars.USE_TELE_MOVE;
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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

@SuppressWarnings("ALL")
public class RobotFuncs {
    public static DcMotorEx ridA;
    public static DcMotorEx ridB;
    public static DcMotorEx extA, extB;
    public static ServoImplEx sextA, sextB;
    public static ServoImplEx sClose, sHeading, sBalans, sMCLaw;
    public static PIDF epd, rpd;
    public static Clown clo;
    public static ThreadedIMU imu = null;
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

    public static enum WAITS {TRANSFER, HOISTER, HOISTERR, EXTENSION, HOISTER_FALL}

    public static SampleMecanumDrive drive;
    public static Encoder leftEncoder;
    public static Encoder rightEncoder;
    public static Encoder frontEncoder;
    public static Lamprey ridlamp;

    static double angDiff(double o1, double o2) {
        return ((o2 - o1 + Math.PI / 2.0) % Math.PI + Math.PI) % Math.PI - Math.PI / 2.0;
    }

    public static void set_grab_pos(int p) {
        conversiePerverssa(SGS - SGD * (p - 1));
        sBalans.setPosition(SBAS - SBAD * (p - 1));
    }

    public static void log_state() {
        TelemetryPacket pack = new TelemetryPacket();
        if (drive != null) {
            pack.put("POSE_x", drive.tl.getPoseEstimate().getX());
            pack.put("POSE_y", drive.tl.getPoseEstimate().getY());
            pack.put("POSE_h", drive.tl.getPoseEstimate().getHeading());
            pack.put("POSE_hdif", angDiff(drive.tl.getPoseEstimate().getHeading(), imu.getLastRead()));

            pack.put("POSEER_x", drive.tl.getLastError().getX());
            pack.put("POSEER_y", drive.tl.getLastError().getY());
            pack.put("POSEER_h", drive.tl.getLastError().getHeading());

            drive.update();
        }
        pack.put("vel", leftEncoder.getCorrectedVelocity());
        pack.put("ver", rightEncoder.getCorrectedVelocity());
        pack.put("vef", frontEncoder.getCorrectedVelocity());
        pack.put("vcl", leftEncoder.getCurrentPosition());
        pack.put("vcr", rightEncoder.getCurrentPosition());
        pack.put("vcf", frontEncoder.getCurrentPosition());
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
        if (USE_TELE_MOVE) {
            pack.put("CUR_leftFront", leftFront.getCurrent(CurrentUnit.MILLIAMPS));
            pack.put("POW_leftFront", leftFront.getPower());
            pack.put("CUR_leftBack", leftBack.getCurrent(CurrentUnit.MILLIAMPS));
            pack.put("POW_leftBack", leftBack.getPower());
            pack.put("CUR_rightFront", rightFront.getCurrent(CurrentUnit.MILLIAMPS));
            pack.put("POW_rightFront", rightFront.getPower());
            pack.put("CUR_rightBack", rightBack.getCurrent(CurrentUnit.MILLIAMPS));
            pack.put("POW_rightBack", rightBack.getPower());
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
            } else if (p == WAITS.HOISTERR) { /// 1: Wait for hoister to the thing
                while (!lom.isStopRequested() && (RMID_POS - ridA.getCurrentPosition()) > MAX_DIF_RID && timeout.seconds() < TIMEOUT) {
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    drive.updatePoseEstimate();
                    log_state();
                    pa = new TelemetryPacket();
                    pa.put("WAIT_FOR_T", RMID_POS - ridA.getCurrentPosition());
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
        } else if (p == WAITS.HOISTERR) { /// 1: Wait for hoister to the thing
            return (RMID_POS - ridA.getCurrentPosition()) < MAX_DIF_RID;
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

    static DcMotorEx initm(String s, boolean e, boolean r, boolean oclock) { /// Init a motor
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, s);

        if (oclock) {
            MotorConfigurationType mconf = m.getMotorType().clone();
            mconf.setAchieveableMaxRPMFraction(1.0);
            m.setMotorType(mconf);
        }

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

    static Thread extT, ridT, cloT, imuT;

    public static void preinit() {
        if (USE_PHOTON) {
            PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            PhotonCore.experimental.setMaximumParallelCommands(8);
            PhotonCore.enable();
            PhotonCore.EXPANSION_HUB.clearBulkCache();
        }
    }

    public static void initma(HardwareMap ch) { /// Init all hardware info
        hardwareMap = ch;

        dashboard = FtcDashboard.getInstance();
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        rightBack = initm("RB", false, true, false);   // P4
        rightFront = initm("RF", false, true, false);  // P2
        leftBack = initm("LB", false, false, false);   // P3
        leftFront = initm("LF", false, false, false);  // P1

        if (useExt) {
            extA = initm("extA", true, false, true);
            extB = initm("extB", true, true, true);
        } else {
            extA = null;
            extB = null;
        }
        if (useRid) {
            ridA = initm("ridA", true, true, true);
            ridB = initm("ridB", true, false, true);
        } else {
            ridA = null;
            ridB = null;
        }
        //underglow = hardwareMap.get(DcMotor.class, "Underglow"); You will not be forgotten

        if (imu == null) {
            imu = new ThreadedIMU(hardwareMap.get(BNO055IMU.class, "imu"));
        }

        sClose = hardwareMap.get(ServoImplEx.class, "sClose");
        sHeading = hardwareMap.get(ServoImplEx.class, "sHeading");
        sBalans = hardwareMap.get(ServoImplEx.class, "sBalans");
        sMCLaw = hardwareMap.get(ServoImplEx.class, "sMCLaw");
        sextA = hardwareMap.get(ServoImplEx.class, "sextA");
        sextB = hardwareMap.get(ServoImplEx.class, "sextB");

        sextA.setPwmEnable();
        sextA.setPwmRange(new PwmControl.PwmRange(505, 2495));
        sextB.setPwmEnable();
        sextB.setPwmRange(new PwmControl.PwmRange(505, 2495));
        /*
        sClose.setPwmEnable();
        sClose.setPwmRange(new PwmControl.PwmRange(505, 2495));
        sHeading.setPwmEnable();
        sHeading.setPwmRange(new PwmControl.PwmRange(505, 2495));
        sBalans.setPwmEnable();
        sBalans.setPwmRange(new PwmControl.PwmRange(505, 2495));
        sMCLaw.setPwmEnable();
        sMCLaw.setPwmRange(new PwmControl.PwmRange(505, 2495));
         */

        conversiePerverssa(SAW);
        conversiePerverssa(SAW - 0.001);
        sBalans.setPosition(SBAG + 0.001);
        conversiePerverssa(SAW + 0.001);
        sBalans.setPosition(SBAG - 0.001);
        conversiePerverssa(SAW - 0.001);
        sBalans.setPosition(SBAG + 0.001);
        conversiePerverssa(SAW + 0.001);
        sBalans.setPosition(SBAG - 0.001);
        conversiePerverssa(SAW - 0.001);
        sBalans.setPosition(SBAG + 0.001);
        conversiePerverssa(SAW + 0.001);
        sBalans.setPosition(SBAG - 0.001);
        conversiePerverssa(SAW);

        sHeading.setPosition(SHG);
        sClose.setPosition(SINCHIS);
        sMCLaw.setPosition(SCO);
        sBalans.setPosition(SBAG);

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, LES));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, RES));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, FES));
        leftEncoder.setDirection(LER ? Encoder.Direction.REVERSE : Encoder.Direction.FORWARD);
        rightEncoder.setDirection(RER ? Encoder.Direction.REVERSE : Encoder.Direction.FORWARD);
        frontEncoder.setDirection(FER ? Encoder.Direction.REVERSE : Encoder.Direction.FORWARD);

        ridlamp = new Lamprey(hardwareMap.get(AnalogInput.class, RIDICARE_LAMPREY));

        if (AUTO_CLOW) {
            sensorRange = hardwareMap.get(DistanceSensor.class, "csensor");
        } else {
            sensorRange = null;
        }

        epd = new PIDF(extA, extB, "Ex", ep, ed, ei, ef, ebp, emd);
        rpd = new PIDF(ridA, ridB, "Ri", rp, rd, ri, rf, rbp, rmd);
        clo = new Clown(sextA, sextB, sHeading, sClose, sMCLaw, sBalans, extA, sensorRange);

        if (imuT == null) {
            imuT = new Thread(imu);
        }
        extT = new Thread(epd);
        ridT = new Thread(rpd);
        cloT = new Thread(clo);
    }

    public static boolean SHOULD_CLOSE_IMU = false;
    public static void startma(LinearOpMode lom, Telemetry tele) { /// Set all values to their starting ones and start the PID threads
        pcoef = 12.0 / batteryVoltageSensor.getVoltage();
        RobotFuncs.lom = lom;
        RobotFuncs.telemetry = tele;

        conversiePerverssa(SAW);
        sHeading.setPosition(SHG);
        sClose.setPosition(SINCHIS);
        sMCLaw.setPosition(SCO);
        sBalans.setPosition(SBAG);

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
        clo.lom = lom;
        cloT.start();

        if (drive != null) {
            drive.startTlt(lom);
        }

        SHOULD_CLOSE_IMU = true;
        imu.setLom(lom);
        imuT.start();
    }

    public static void opcl() {
        if (epsEq(sextA.getPosition(), SAW) && epsEq(sClose.getPosition(), SINCHIS)) {
            clo.clw.reset();
            clo._clw = false;
        }
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
        sextA.setPwmDisable();
        sextB.setPwmDisable();
        sClose.setPwmDisable();
        sHeading.setPwmDisable();
        sBalans.setPwmDisable();
        sMCLaw.setPwmDisable();
        ridlamp.close();
        TelemetryPacket tp = new TelemetryPacket();
        tp.put("SHOULD_CLOSE_IMU", SHOULD_CLOSE_IMU);
        dashboard.sendTelemetryPacket(tp);
        if (SHOULD_CLOSE_IMU) {
            SHOULD_CLOSE_IMU = false;
            imu.close();
            imu = null;
        }
        if (ridA != null) {
            ridA.setPower(0);
            ridB.setPower(0);
        }
        if (extA != null) {
            extA.setPower(0);
            extB.setPower(0);
        }
        if (sensorRange != null) {
            sensorRange.close();
        }
        if (drive != null) {
            drive.joinTlt();
        }
        batteryVoltageSensor.close();
        try {
            imuT.join();
            extT.join();
            ridT.join();
            cloT.join();
            imuT = null;
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
