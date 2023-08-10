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

import static org.firstinspires.ftc.teamcode.RobotVars.CU_TESTING;
import static org.firstinspires.ftc.teamcode.RobotVars.EMIN;
import static org.firstinspires.ftc.teamcode.RobotVars.RMIU_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.RTOP_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.SAG;
import static org.firstinspires.ftc.teamcode.RobotVars.SAP;
import static org.firstinspires.ftc.teamcode.RobotVars.SAW;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAG;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAP;
import static org.firstinspires.ftc.teamcode.RobotVars.SCC;
import static org.firstinspires.ftc.teamcode.RobotVars.SCO;
import static org.firstinspires.ftc.teamcode.RobotVars.SDESCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.SHG;
import static org.firstinspires.ftc.teamcode.RobotVars.SHITTY_WORKAROUND_POWER;
import static org.firstinspires.ftc.teamcode.RobotVars.SHITTY_WORKAROUND_TIME;
import static org.firstinspires.ftc.teamcode.RobotVars.SHP;
import static org.firstinspires.ftc.teamcode.RobotVars.SINCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.USE_TELE;
import static org.firstinspires.ftc.teamcode.RobotVars.coneClaw;
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
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.DONT_KILL_IMU;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.SHOULD_CLOSE_IMU;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.batteryVoltageSensor;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.clo;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.conversiePerverssa;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.dashboard;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.endma;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.epd;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.epsEq;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.ext;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.extA;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.extB;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.imu;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.initma;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.leftBack;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.leftFront;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.log_state;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.opcl;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.preinit;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.rid;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.ridA;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.rightBack;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.rightFront;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.rpd;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sBalans;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sClose;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sHeading;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sMCLaw;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.set_grab_pos;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sextA;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.spe;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.startma;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@SuppressWarnings({"SpellCheckingInspection", "CommentedOutCode"})
@Config
@TeleOp
public class OP_Mode_mk3 extends LinearOpMode {
    boolean L2A = false;
    boolean L2B = false;
    boolean L2Y = false;
    boolean L2U = false;
    boolean L2D = false;
    boolean L2R = false;
    boolean G2X = false;
    boolean R2RB = false;
    boolean R2LB = false;
    boolean G2LT = false;
    boolean G2L = false;
    boolean RB = false;
    boolean R1X = false;
    boolean R1Y = false;
    boolean R1A = false;
    boolean R1B = false;
    boolean R1DD = false;
    boolean R1DU = false;

    public static int RID_POS = 0;
    public static int oldpos;

    public double lep, led, lei, lef, lebp;
    public double rep, red, rei, ref, rebp;

    public static double P1 = 1;
    public static double P2 = 1;
    public static double P3 = 1;
    public static double P4 = 1;
    public static double XC = 1;
    public static double YC = 1;
    public static double TURP = 0.2;
    int i;

    ElapsedTime etimer = new ElapsedTime(1000);
    ElapsedTime rtimer = new ElapsedTime(1000);
    ElapsedTime SHITTY_WORKAROUND_TIMER = new ElapsedTime(0);
    boolean SHITTY_WORKAROUND_TIMED = false;
    public static double EXTL = 0.2;
    public static double RIDL = 0.2;
    ElapsedTime tpi = new ElapsedTime(0);

    double gdif(double lr) {
        if (lr < Math.PI) {
            return -lr;
        } else {
            return 2 * Math.PI - lr;
        }
    }

    public static double turnLastError = 0;
    double sign(double error) {
        if (error == 0) {
            return 0;
        } else if (error > 0) {
            return 1;
        } else {
            return -1;
        }
    }

    public static double TURNP = 0.5;
    public static double TURND = 0.09;
    public static double TURNF = 0.15;
    public static double MAX_ERROR = 0.03;

    double get_turn() {
        double lr = imu.getLastRead();
        double error = gdif(lr);
        double derivate = (error - turnLastError) / tpi.seconds();
        TelemetryPacket telp = new TelemetryPacket();
        telp.put("GET_TURN_LR", lr);
        telp.put("GET_TURN_ERROR", error);
        telp.put("GET_TURN_DERIVATE", derivate);
        dashboard.sendTelemetryPacket(telp);
        turnLastError = error;
        if (Math.abs(error) > MAX_ERROR) {
            return error * TURNP + derivate * TURND + sign(error) * TURNF;
        } else {
            return 0;
        }
    }

    /*
       /__\
       |___|
       |___|
       |_I_|
       |___|
 _____ _____ _____
|__I__|_____|__R__|
       |___|
       |___|
       |___|
       |_N_|
       |___|
       |___|
       |___|
       |___|
  Doamne Fereste!
     */

    public void runOpMode() {
        preinit();
        L2A = L2B = L2Y = L2U = L2D = G2X = R2RB = R2LB = RB = R1B = R1X = R1Y = R1A = R1DD = R1DU = L2R = coneReady = false;
        oldpos = RID_POS;
        DONT_KILL_IMU = true;

        RobotFuncs.drive = new SampleMecanumDrive(hardwareMap);
        initma(hardwareMap);
        /*
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
         */
        //RobotFuncs.drive = null;

        waitForStart();

        startma(this, telemetry);

        ElapsedTime timer = new ElapsedTime(0);
        /*ElapsedTime g1t = new ElapsedTime(0);
        double lhpos = 0;*/
        lep = ep;
        lei = ei;
        led = ed;
        lef = ef;
        lebp = ebp;
        rep = rp;
        rei = ri;
        red = rd;
        ref = rf;
        rebp = rbp;
        timer.reset();
        SHITTY_WORKAROUND_TIMER.reset();
        tpi.reset();
        while (opModeIsActive()) {
            if (oldpos != RID_POS) {
                rpd.set_target(RID_POS, 0);
                oldpos = RID_POS;
            }
            if (lep != ep || lei != ei || led != ed || lef != ef || lebp != ebp) {
                epd.update_pid(ep, ei, ed, ef, ebp);
                lep = ep;
                lei = ei;
                led = ed;
                lebp = ebp;
            }
            if (rep != rp || rei != ri || red != rd || ref != rf || rebp != rbp) {
                rpd.update_pid(rp, ri, rd, rf, rbp);
                rep = rp;
                rei = ri;
                red = rd;
                rebp = rbp;
            }

            if (CU_TESTING == 1) {
                conversiePerverssa(SAP);
                sHeading.setPosition(SHP);
                sMCLaw.setPosition(SCO);
                sBalans.setPosition(SBAP);
            } else if (CU_TESTING == 2) {
                conversiePerverssa(SAG);
                sHeading.setPosition(SHG);
                sMCLaw.setPosition(SCC);
                sBalans.setPosition(SBAG);
            }

            final double speed = Math.hypot(gamepad1.left_stick_x * XC, gamepad1.left_stick_y * YC);
            final double angle = Math.atan2(gamepad1.left_stick_y * YC, gamepad1.left_stick_x * XC) - Math.PI / 4;// + ch;

            double turn;
            if (gamepad1.left_trigger > 0.5 && Math.abs(gamepad1.right_stick_x) < 0.05) {
                turn = get_turn();
            } else {
                turn = -gamepad1.right_stick_x;
            }
            tpi.reset();
            final double ms = speed * Math.sin(angle);
            final double mc = speed * Math.cos(angle);

            final double lfPower = ms + turn;
            final double rfPower = mc - turn;
            final double lbPower = mc + turn;
            final double rbPower = ms - turn;

            if (!L2A && gamepad2.a) {
                rid(RTOP_POS);
            }
            L2A = gamepad2.a;

            /*if (!L2U && gamepad2.dpad_up) {
                rid(RMID_POS);
            }
            L2U = gamepad2.dpad_up;*/

            if (!L2D && gamepad2.dpad_down) {
                rid(RMIU_POS);
            }
            L2D = gamepad2.dpad_down;

            if (!L2B && (gamepad2.b || gamepad1.right_bumper)) {
                ext(EMIN);
            }
            L2B = gamepad2.b || gamepad1.right_bumper;

            if (!G2L && gamepad2.dpad_left) {
                sMCLaw.setPosition(SCC);
                coneReady = true;
            }
            G2L = gamepad2.dpad_left;

            if (!L2Y && (gamepad2.y || gamepad1.dpad_right)) {
                clo.toPut = true;
            }
            L2Y = (gamepad2.y || gamepad1.dpad_right);

            if (!R2LB && gamepad2.left_bumper) {
                opcl();
                if (!clo.rtg) {
                    clo.toGet = true;
                } else {
                    clo.toPrepCone = true;
                }
            }
            R2LB = gamepad2.left_bumper;

            if (!R1DD && gamepad1.dpad_down) {
                conversiePerverssa(SAG);
                clo.toGet = true;
            }
            R1DD = gamepad1.dpad_down;

            if (!R1DU && gamepad1.dpad_up) {
                clo.toPrepCone = true;
            }
            R1DU = gamepad1.dpad_up;

            if (!L2R && gamepad2.dpad_right) {
                sMCLaw.setPosition(SCO);
            }
            L2R = gamepad2.dpad_right;

            if (!R2RB && gamepad2.right_bumper && !coneClaw) {
                clo.rtg = false;
                conversiePerverssa(SAW);
                sBalans.setPosition(SBAG);
                sClose.setPosition(SINCHIS);
                sHeading.setPosition(SHG);
                conversiePerverssa(SAW);
            }
            R2RB = gamepad2.right_bumper;

            if (!R1Y && gamepad1.y) {
                opcl();
                set_grab_pos(1);
            }
            R1Y = gamepad1.y;

            if (!R1B && gamepad1.b) {
                opcl();
                set_grab_pos(2);
            }
            R1B = gamepad1.b;

            if (!R1A && gamepad1.a && !gamepad1.start) {
                opcl();
                set_grab_pos(3);
            }
            R1A = gamepad1.a;

            if (!R1X && gamepad1.x) {
                opcl();
                set_grab_pos(4);
            }
            R1X = gamepad1.x;

            final double DPC = 1 - 0.6 * gamepad2.right_trigger;
            if (Math.abs(gamepad2.left_stick_y) > 0.001) {
                if (!clo.cput) {
                    spe(false, -gamepad2.left_stick_y * DPC);
                }
                etimer.reset();
            } else {
                if (SHITTY_WORKAROUND_TIMER.seconds() >= SHITTY_WORKAROUND_TIME) {
                    spe(false, 0);
                }
            }

            if (extA != null && etimer.seconds() < EXTL) {
                epd.set_target(extA.getCurrentPosition(), 0);
            }

            if (Math.abs(gamepad2.right_stick_y) > 0.001) {
                if (!clo.cput) {
                    spe(true, -gamepad2.right_stick_y * DPC);
                }
                rtimer.reset();
            } else {
                    spe(true, 0);
            }

            if (ridA != null && rtimer.seconds() < RIDL) {
                rpd.set_target(ridA.getCurrentPosition(), 0);
            }

            if (!G2X && (gamepad2.x || gamepad1.left_bumper)) {
                telemetry.addData("CLIC X", i);
                if (epsEq(sClose.getPosition(), SINCHIS)) {
                    sClose.setPosition(SDESCHIS);
                    coneClaw = false;
                } else {
                    sClose.setPosition(SINCHIS);
                    coneClaw = true;
                }
                ++i;
            }
            G2X = gamepad2.x || gamepad1.left_bumper;

            if (!G2LT && gamepad2.left_trigger > 0.9) {
                if (extA != null) {
                    extA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    extA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    extB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    extB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if (ridA != null) {
                    ridA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    ridA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                clo.toPrepCone = clo.cprepCone = clo.cput = clo.toPut = clo.chput = clo.cget = false;
                epd.set_target(0, 0);
                rpd.set_target(0, 0);
                sClose.setPosition(SDESCHIS);
                conversiePerverssa(SAG);
                sHeading.setPosition(SHG);
                sBalans.setPosition(SBAG);

                telemetry.addLine("RESET EVERYTHING!");
                telemetry.update();
            }
            G2LT = gamepad2.left_trigger > 0.9;

            if (USE_TELE) {
                TelemetryPacket pack = new TelemetryPacket();
                pack.put("CycleTime", timer.milliseconds());
                log_state();
                dashboard.sendTelemetryPacket(pack);
                timer.reset();
            }

            if (extA != null) {
                if (SHITTY_WORKAROUND_TIMER.seconds() < SHITTY_WORKAROUND_TIME) {
                    epd.curRet = true;
                    epd.use = false;
                    extA.setPower(-SHITTY_WORKAROUND_POWER);
                    extB.setPower(-SHITTY_WORKAROUND_POWER);
                } else if (!SHITTY_WORKAROUND_TIMED) {
                    SHITTY_WORKAROUND_TIMED = true;
                    epd.use = true;
                    epd.curRet = false;
                    extA.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    extA.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    extB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    extB.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                }
            }

            if (gamepad2.left_trigger > 0.7 && gamepad2.right_trigger > 0.7) {
                break;
            }

            pcoef = 12.0 / batteryVoltageSensor.getVoltage();
            final double spcoef = 1 - 0.65 * gamepad1.right_trigger;
            final double fcoef = pcoef * spcoef;
            leftFront.setPower(lfPower * fcoef * P1);
            rightFront.setPower(rfPower * fcoef * P2);
            leftBack.setPower(lbPower * fcoef * P3);
            rightBack.setPower(rbPower * fcoef * P4);
        }

        boolean KILL = false;

        endma();

        while (!isStopRequested()) {
            if (!KILL && gamepad2.left_trigger < 0.2 && gamepad2.right_trigger < 0.2) {
                KILL = true;
            } else if (KILL && gamepad2.left_trigger > 0.7 && gamepad2.right_trigger > 0.7) {
                runOpMode();
            }
            sleep(5);
        }
    }
}
