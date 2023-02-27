package org.firstinspires.ftc.teamcode.mk3;

/*
         ____    _    ____ ___    _      _ _____  ___   ___
        |  _ \  / \  / ___|_ _|  / \    / |___ / / _ \ / _ \
        | | | |/ _ \| |    | |  / _ \   | | |_ \| | | | | | |
        | |_| / ___ \ |___ | | / ___ \  | |___) | |_| | |_| |
        |____/_/   \_\____|___/_/   \_\ |_|____/ \___/ \___/

         ____   ___  _____ _____      _    ____      _    ____  _____ ____   ____
        / ___| / _ \|  ___|_   _|    / \  |  _ \    / \  | __ )| ____/ ___| / ___|
        \___ \| | | | |_    | |     / _ \ | |_) |  / _ \ |  _ \|  _| \___ \| |
        ___) | |_| |  _|   | |    / ___ \|  _ <  / ___ \| |_) | |___ ___) | |___
        |____/ \___/|_|     |_|   /_/   \_\_| \_\/_/   \_\____/|_____|____/ \____|
        ____  _____ ______     _______ ____  ____
        |  _ \| ____|  _ \ \   / / ____|  _ \/ ___|
        | |_) |  _| | |_) \ \ / /|  _| | |_) \___ \
        |  __/| |___|  _ < \ V / | |___|  _ < ___) |
        |_|   |_____|_| \_\ \_/  |_____|_| \_\____/

         _____   ____  _____ ______     _______ ____  ____
        | ____| |  _ \| ____|  _ \ \   / / ____|  _ \/ ___|
        |  _|   | |_) |  _| | |_) \ \ / /|  _| | |_) \___ \
        | |___  |  __/| |___|  _ < \ V / | |___|  _ < ___) |
        |_____| |_|   |_____|_| \_\ \_/  |_____|_| \_\____/
         ____  _   _ ____  _____      _    ____  _____ ____  _____ _   _ _____  _
        |  _ \| | | |  _ \| ____|    / \  |  _ \| ____|  _ \| ____| \ | |_   _|/ \
        | |_) | | | | |_) |  _|     / _ \ | | | |  _| | |_) |  _| |  \| | | | / _ \
        |  _ <| |_| |  __/| |___   / ___ \| |_| | |___|  _ <| |___| |\  | | |/ ___ \
        |_| \_\\___/|_|   |_____| /_/   \_\____/|_____|_| \_\_____|_| \_| |_/_/   \_\
 */

/*
 * SOFT ARABESC PERVERS VERSIUNE 0.31δ
 * AUTOR: VERICU
 * E PERVERS, RUPE ADERENTA
 *
 * Cu de toate fara ceapa boss
 */

import static org.firstinspires.ftc.teamcode.RobotVars.EMAX;
import static org.firstinspires.ftc.teamcode.RobotVars.EMIN;
import static org.firstinspires.ftc.teamcode.RobotVars.RBOT_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.RMID_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.RMIU_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.RTOP_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.S1PO;
import static org.firstinspires.ftc.teamcode.RobotVars.S2PO;
import static org.firstinspires.ftc.teamcode.RobotVars.S3PO;
import static org.firstinspires.ftc.teamcode.RobotVars.SCC;
import static org.firstinspires.ftc.teamcode.RobotVars.SCO;
import static org.firstinspires.ftc.teamcode.RobotVars.SDESCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.SINCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.USE_PHOTON;
import static org.firstinspires.ftc.teamcode.RobotVars.USE_TELE;
import static org.firstinspires.ftc.teamcode.RobotVars.coneReady;
import static org.firstinspires.ftc.teamcode.RobotVars.pcoef;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@SuppressWarnings({"CommentedOutCode", "SpellCheckingInspection"})
@Config
@TeleOp
public class OP_Mode_mk3 extends LinearOpMode {
    //sasiu
    public DcMotorEx leftBack;
    public DcMotorEx leftFront;
    public DcMotorEx rightBack;
    public DcMotorEx rightFront;
    //restu
    public DcMotorEx ridA, ridB;
    public DcMotorEx extA, extB;
    public Servo sextA, sextB;
    public Servo sClose, sHeading, sMCLaw;

    boolean L2A = false;
    boolean L2RB = false;
    boolean L2B = false;
    boolean L2Y = false;
    boolean L2U = false;
    boolean L2D = false;
    boolean G2X = false;
    boolean R2RB = false;
    boolean R2LB = false;
    boolean R2LT = false;
    boolean RB = false;
    boolean switched = false;

    public static int dif = 170;
    public static double headP = 1.2;

    public static double UPPS = 100;
    double UPP = 100;
    public static double UPPP = 100;

    public static double ep = 0;
    public static double ed = 0;
    public static double ei = 0;
    public static double ebp = 0;
    public double lep, led, lei, lebp;

    public static double rp = 0;
    public static double rd = 0;
    public static double ri = 0;
    public static double rbp = 0;
    public double rep, red, rei, rebp;

    public static double P1 = 1;
    public static double P2 = 1;
    public static double P3 = 1;
    public static double P4 = 1;

    /* You will not be forgotten
    DcMotor underglow;
    ElapsedTime et = new ElapsedTime(0);
    double luv = 0;
    public double FC = 0.5;
    public double SPC = 1.2;
    public double MINP = 0.2;
    void upd_underglow(double speed) { /// Underglow cam pervers
        double cs = Math.abs(speed) * Math.sqrt(2) / 2 * SPC;
        if (cs > luv) {
            luv = cs;
        } else {
            luv -= et.seconds() * FC;
        }
        luv = Math.min(Math.max(luv, MINP), 1);

        underglow.setPower(-luv);
        et.reset();
    }
     */

    public double WT = 0.2;
    public static double HMIN = 0.005;

    PIDF epd, rpd;
    Transf tra;

    void ep(double p) {
        extA.setPower(p);
        extB.setPower(p);
    }

    void rp(double p) {
        ridA.setPower(p);
        ridB.setPower(p);
    }

    void spe(boolean er, double p) {
        if (!er) {
            if (p == 0) {
                epd.use = true;
            } else {
                epd.use = false;
                if (extA.getCurrentPosition() < EMIN) {
                    ep(0);
                    return;
                }
                if (extA.getCurrentPosition() > EMAX) {
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
                if (ridA.getCurrentPosition() < RBOT_POS) {
                    rp(0);
                    return;
                }
                if (ridA.getCurrentPosition() > RTOP_POS) {
                    rp(0);
                    return;
                }
                rp(p);
            }
        }
    }

    void rid(int pos) {
        if (coneReady || pos == RBOT_POS) {
            if (pos == RBOT_POS) {
                sMCLaw.setPosition(SCO);
                coneReady = false;
            }
            rpd.set_target(pos, false);
        } else {
            tra.toGet = true;
        }
    }

    void ext(int pos) {
        if (pos == EMAX) {
            tra.toGet = true;
            coneReady = false;
        } else {
            sClose.setPosition(SCC);
        }
        rid(RBOT_POS);
        epd.set_target(pos, false);
    }

    DcMotorEx initm(String s, boolean e, boolean r) {
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

    public void runOpMode() {
        if (USE_PHOTON) {
            PhotonCore.enable();
            PhotonCore.experimental.setSinglethreadedOptimized(false);
        }
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        L2A = L2B = L2Y = L2U = L2D = G2X = R2RB = R2LB = R2LT = RB = switched = coneReady = false;
        UPP = UPPS;

        rightBack = initm("RB", false, false);
        rightFront = initm("RF", false, false);
        leftBack = initm("LB", false, true);
        leftFront = initm("LF", false, true);
        extA = initm("extA", true, true);
        extB = initm("extB", true, true);
        ridA = initm("ridA", true, true);
        ridB = initm("ridB", true, true);
        //underglow = hardwareMap.get(DcMotor.class, "Underglow"); You will not be forgotten

        sClose = hardwareMap.get(Servo.class, "gho");
        sHeading = hardwareMap.get(Servo.class, "ghh");
        sMCLaw = hardwareMap.get(Servo.class, "ghc");
        sextA = hardwareMap.get(Servo.class, "sextA");
        sextB = hardwareMap.get(Servo.class, "sextB");

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        /*parameters.mode = BNO055IMU.SensorMode.COMPASS;
        parameters.gyroPowerMode = BNO055IMU.GyroPowerMode.FAST;
        parameters.gyroBandwidth = BNO055IMU.GyroBandwidth.HZ523; /// TODO ???????
        parameters.gyroRange = BNO055IMU.GyroRange.DPS2000;*/
        imu.initialize(parameters);

        epd = new PIDF(extA, extB, ep, ed, ei, ebp);
        rpd = new PIDF(ridA, ridB, rp, rd, ri, rbp);
        tra = new Transf(sextA, sextB, sHeading, sClose, sMCLaw, extA);

        Thread extT = new Thread(epd);
        Thread ridT = new Thread(rpd);
        Thread traT = new Thread(tra);

        Servo S1 = hardwareMap.get(Servo.class, "SPe");
        Servo S2 = hardwareMap.get(Servo.class, "SPa1");
        Servo S3 = hardwareMap.get(Servo.class, "SPa2");
        S1.setPosition(S1PO);
        S2.setPosition(S2PO);
        S3.setPosition(S3PO);

        waitForStart();

        pcoef = 12.0 / batteryVoltageSensor.getVoltage();

        epd.shouldClose = false;
        epd.use = true;
        epd.target = 0;
        extT.start();

        rpd.shouldClose = false;
        rpd.use = true;
        rpd.target = 0;
        ridT.start();

        tra.shouldClose = true;
        traT.start();


        ElapsedTime timer = new ElapsedTime(0);
        ElapsedTime g1t = new ElapsedTime(0);
        double lhpos = 0;
        lep = ep;
        lei = ei;
        led = ed;
        lebp = ebp;
        rep = rp;
        rei = ri;
        red = rd;
        rebp = rbp;
        while (opModeIsActive()) {
            if (lep != ep || lei != ei || led != ed || lebp != ebp) {
                epd.update_pid(ep, ei, ed, ebp);
                lep = ep;
                lei = ei;
                led = ed;
                lebp = ebp;
            }
            if (rep != rp || rei != ri || red != rd || rebp != rbp) {
                rpd.update_pid(rp, ri, rd, rbp);
                rep = rp;
                rei = ri;
                red = rd;
                rebp = rbp;
            }

            if (USE_TELE) {
                TelemetryPacket fp = new TelemetryPacket();
                fp.put("CycleTime", timer.milliseconds());
                timer.reset();
                dashboard.sendTelemetryPacket(fp);
            }

            final double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double ch = imu.getAngularOrientation().firstAngle;
            final double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;// + ch;
            double hdif = 0;
            if (Math.abs(gamepad1.right_stick_x) > 0.001) {
                g1t.reset();
            }
            if (g1t.seconds() < WT) {
                lhpos = ch;
            } else {
                hdif = lhpos - ch;
                if (hdif > Math.PI / 2) {
                    hdif = Math.PI - hdif;
                } else if (hdif < -Math.PI / 2) {
                    hdif -= Math.PI;
                }
                if (Math.abs(hdif) < HMIN || speed > 0.01) {
                    hdif = 0;
                }
            }

            final double turn = hdif * headP - gamepad1.right_stick_x;
            final double ms = speed * Math.sin(angle);
            final double mc = speed * Math.cos(angle);

            final double lfPower = ms + turn;
            final double rfPower = mc - turn;
            final double lbPower = mc + turn;
            final double rbPower = ms - turn;
            //upd_underglow(Math.abs(turn) + Math.abs(speed));

            if (!L2RB && gamepad2.right_bumper) {
                UPP += UPPP;
            }
            L2RB = gamepad2.right_bumper;

            if (!L2A && gamepad2.a) {
                rid(RTOP_POS);
            }
            L2A = gamepad2.a;
            if (!L2U && gamepad2.dpad_up) {
                rid(RMID_POS);
            }
            L2U = gamepad2.dpad_up;
            if (!L2D && gamepad2.dpad_down) {
                rid(RMIU_POS);
            }
            L2D = gamepad2.dpad_down;
            if (!L2B && gamepad2.b) {
                rid(RBOT_POS);
                ext(EMIN);
            }
            L2B = gamepad2.b;

            if (!L2Y && gamepad2.y) {
                tra.toPut = true;
            }
            L2Y = gamepad2.y;

            if (!R2RB && gamepad2.right_bumper) {
                rid(RBOT_POS);
                ext(EMAX);
            }
            R2RB = gamepad2.right_bumper;

            final double DPC = 1 - 0.6 * gamepad2.right_trigger;
            if (Math.abs(gamepad2.right_stick_y) > 0.001) {
                spe(false, gamepad2.right_stick_y * DPC);
            } else {
                spe(false, 0);
            }

            if (Math.abs(gamepad2.left_stick_y) > 0.001) {
                spe(true, gamepad2.left_stick_y * DPC);
            } else {
                spe(true, 0);
            }

            if (!RB && gamepad2.left_bumper) {
                rpd.set_target(rpd.target - dif, true);
            }
            if (RB && !gamepad2.left_bumper) {
                rpd.set_target(rpd.target + dif, false);
            }
            RB = gamepad2.left_bumper;

            if (!G2X && gamepad2.x) {
                if (switched) {
                    sClose.setPosition(SDESCHIS);
                } else {
                    sClose.setPosition(SINCHIS);
                }
                switched = !switched;
            }
            G2X = gamepad2.x;

            //dam blana in motoare
            pcoef = 12.0 / batteryVoltageSensor.getVoltage();
            final double spcoef = 1 - 0.6 * gamepad1.right_trigger;
            final double fcoef = pcoef * spcoef;
            leftFront.setPower(lfPower * fcoef * P1);  // LB
            rightFront.setPower(rfPower * fcoef * P2); // LF
            leftBack.setPower(lbPower * fcoef * P3);   // RB
            rightBack.setPower(rbPower * fcoef * P4);  // RF
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        epd.shouldClose = true;
        rpd.shouldClose = true;
        tra.shouldClose = true;
        try {
            extT.join(10);
            ridT.join(10);
            traT.join(10);
        } catch (Exception e) {
            e.printStackTrace();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}
