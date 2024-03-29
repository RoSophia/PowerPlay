package org.firstinspires.ftc.teamcode.mk3;

import static org.firstinspires.ftc.teamcode.RobotVars.EMIN;
import static org.firstinspires.ftc.teamcode.RobotVars.SAG;
import static org.firstinspires.ftc.teamcode.RobotVars.SAH;
import static org.firstinspires.ftc.teamcode.RobotVars.SAP;
import static org.firstinspires.ftc.teamcode.RobotVars.SAW;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAAP;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAC;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAG;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAH;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAP;
import static org.firstinspires.ftc.teamcode.RobotVars.SCC;
import static org.firstinspires.ftc.teamcode.RobotVars.SCM;
import static org.firstinspires.ftc.teamcode.RobotVars.SCO;
import static org.firstinspires.ftc.teamcode.RobotVars.SDESCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.SGS;
import static org.firstinspires.ftc.teamcode.RobotVars.SHAP;
import static org.firstinspires.ftc.teamcode.RobotVars.SHG;
import static org.firstinspires.ftc.teamcode.RobotVars.SHP;
import static org.firstinspires.ftc.teamcode.RobotVars.SINCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.SMDESCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.armHolding;
import static org.firstinspires.ftc.teamcode.RobotVars.coneClaw;
import static org.firstinspires.ftc.teamcode.RobotVars.coneReady;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.conversiePerverssa;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.dashboard;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.epd;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.epsEq;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.ext;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.extA;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.extB;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.opcl;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.ridlamp;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sClose;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sMCLaw;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sextA;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sextB;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.spe;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.List;

@SuppressWarnings("ALL")
@Config
public class Clown implements Runnable {
    public static int MIP = 100;
    public static double ME = 5;
    public static double ETC = 0.14;
    public static double CPT = 0.0;
    public static double CET = 0.0;
    public static double CHT = 0.0;
    public static double WPT = 0.05;
    public static double WET = 0.0;
    public static double WHT = 0.2;
    public static double TTT = 0.0;
    public static boolean CLAW = false;
    public static double PUTC = 1.212;
    public static double UPTC = 1.212;
    public static double PREPC = 1.212;
    public static double CIP = 0.11;
    public static double GB = 0.20;
    public static double GHT = 0.1;
    public static double WT = 0.14;
    public static double CD = 0.00;
    public static double CB = 0.05;
    public static double CC = 0.1;
    public static double ED = 0.15;
    public static double CDIST = 200;
    public static boolean AUTO_CLOW = false;
    public static double RET_POW = -0.2;
    public static double RLAMP_MPOS = 0.5;
    public static double RLAMP_POS = 1.12;
    public static boolean USE_TELE = true;
    public static double CLWT = 0.08;

    public static LinearOpMode lom;

    private Servo sa, sb, sHeading, sClaw, sBalans, sMClaw;
    private DcMotorEx ce;
    public boolean cput = false;
    public boolean cget = false;
    public boolean cprepCone = false;
    public boolean toPut = false;
    public boolean toGet = false;
    public boolean rtg = false;
    public boolean shouldClose = false;
    public boolean toPrepCone = false;
    public boolean tppc = false;
    public boolean waiting = true;
    public boolean hput = false;
    public boolean chput = false;
    private boolean _rlamp = false;
    private boolean _rrlamp = false;
    private boolean _cba = false;
    private boolean _cmc = false;
    private boolean _csc = false;
    private boolean _csp = false;
    private boolean _csw = false;
    public boolean _clw = true;
    public ElapsedTime clw = new ElapsedTime(0);

    public static double OPP = 0.0;
    public static double OEP = 0.0;
    public static double OHP = 0.0;

    public DistanceSensor csensor;
    ElapsedTime ht = new ElapsedTime(0);

    List<Double> tims = Arrays.asList(CPT, CET, CHT);
    List<Double> wtim = Arrays.asList(WPT, WET, WHT);
    List<Double> poff = Arrays.asList(OPP, OEP, OHP);
    int timt = 0;

    ElapsedTime et = new ElapsedTime(0);
    ElapsedTime ct = new ElapsedTime(0);

    public void reset() {
        cput = false;
        cget = false;
        cprepCone = false;
        toPut = false;
        toGet = false;
        rtg = false;
        shouldClose = false;
        toPrepCone = false;
        tppc = false;
        waiting = true;
        hput = false;
        chput = false;
        _rlamp = false;
        _rrlamp = false;
        _cmc = false;
        _cba = false;
        _csc = false;
        _csp = false;
        _csw = false;
        _clw = true;
    }

    public Clown(Servo sa, Servo sb, Servo sHeading, Servo sClaw, Servo sMClaw, Servo sBalans, DcMotorEx ce, DistanceSensor csensor) {
        reset();
        this.sa = sa;
        this.sb = sa;
        this.sHeading = sHeading;
        this.sClaw = sClaw;
        this.sMClaw = sMClaw;
        this.sBalans = sBalans;
        this.ce = ce;
        this.csensor = csensor;
    }

    double sp;
    double DT = 1.212;
    ElapsedTime gtim = new ElapsedTime(0);
    boolean AAAAAAAAAAAAA = false;
    boolean BBBBBBBBBBBBB = false;


    public static boolean NOT_USE_LAMP = true;
    ElapsedTime kmskms = new ElapsedTime(0);
    public static double KMSKMS1 = 0.2;
    public static double KMSKMS2 = 0.4;

    public void run() {
        while (!shouldClose && !lom.isStopRequested()) {
            double cd = 0;
            if (AUTO_CLOW && csensor != null) {
                cd = csensor.getDistance(DistanceUnit.MM);
            }

            if (USE_TELE) {
                TelemetryPacket packet = new TelemetryPacket();
                tims = Arrays.asList(CPT, CET, CHT);
                wtim = Arrays.asList(WPT, WET, WHT);
                poff = Arrays.asList(OPP, OEP, OHP);

                packet.put("cput", cput);
                packet.put("cget", cget);
                packet.put("cpre", cprepCone);
                packet.put("tpre", toPrepCone);
                packet.put("tput", toPut);
                packet.put("tget", toGet);
                packet.put("tppc", tppc);
                packet.put("cred", coneReady);
                packet.put("ahol", armHolding);
                packet.put("ccla", coneClaw);
                packet.put("DT", DT);
                packet.put("timt", timt);
                packet.put("ctim", tims.get(timt));
                packet.put("SER_SBA", sBalans.getPosition());
                packet.put("SER_SEA", sextA.getPosition());
                packet.put("SER_SEB", sextB.getPosition());
                packet.put("SER_SEH", sHeading.getPosition());
                packet.put("SER_SEC", sClose.getPosition());
                packet.put("SER_LAM", ridlamp.getPosition());
                packet.put("csens", cd);

                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }

            /*
             * This whole class works by the principle of saying what you want to do,
             * and it trying to get the internal state of the robot to the appropriate one to do what you need.
             */

            if (AUTO_CLOW && cd < CDIST && cd > 0 && !coneClaw && !coneReady && !cput && !cget && !cprepCone) {
                toPut = true;
            }

            if (CLAW) { // Used only in testing
                sClaw.setPosition(SINCHIS);
                CLAW = false;
                coneReady = true;
            }

            if (cput && toPut) { // If you want to put but are already putting, cancel the request
                toPut = false;
            }

            if (coneClaw && toPut && (ce == null || ce.getCurrentPosition() < MIP)) { // If you want to put, you have a cone in your claw and are not extended too far out
                // Start the putting sequence
                tppc = false;
                toPut = false;
                cput = true;
                toPrepCone = false;
                cprepCone = false;
                cget = false;
                rtg = false;
                sp = sa.getPosition();

                et.reset();
                ext(EMIN);
                //sHeading.setPosition(SHP);
                sClaw.setPosition(SINCHIS);
                sMClaw.setPosition(SCO);
                sBalans.setPosition(SBAP);
                sHeading.setPosition(SHP);
                _rlamp = false;
                _rrlamp = false;
                _cmc = false;
                _cba = false;
                _csc = false;
                _csp = false;
                _csw = false;
                kmskms.reset();
            }

            if (cput) { // Currently putting
                epd.curRet = true;
                extA.setPower(RET_POW);
                extB.setPower(RET_POW);

                if (et.seconds() >= wtim.get(timt) * DT && !_csp) {
                    conversiePerverssa(SAP + poff.get(timt));
                    _csp = true;
                } else if (_csp) {
                    if (((!NOT_USE_LAMP && ridlamp.getPosition() >= RLAMP_MPOS) || (NOT_USE_LAMP && kmskms.seconds() > KMSKMS1)) && !_rlamp && !_rrlamp) {
                        sMCLaw.setPosition(SCM);
                        _rrlamp = true;
                    }
                    if (((!NOT_USE_LAMP && ridlamp.getPosition() >= RLAMP_POS) || (NOT_USE_LAMP && kmskms.seconds() > KMSKMS2)) && !_rlamp) {
                        _rlamp = true;
                        et.reset();
                    } else if (_rlamp) {
                        //spe(true, RET_POW / 2);
                        if (et.seconds() > (tims.get(timt) + CD) * DT && !_cmc) { /// Close the mini servo to keep the cone in the holding bay
                            sMClaw.setPosition(SCC);
                            _cmc = true;
                        }
                        if (et.seconds() > (tims.get(timt) + CD + CC) * DT && !_csc) { /// Open the claw after it has reached the holding bay
                            sClaw.setPosition(SMDESCHIS);
                            _csc = true;
                        }
                        if (et.seconds() > (tims.get(timt) + CD + CC + CB) * DT && !_cba) { /// Open the claw after it has reached the holding bay
                            sHeading.setPosition(SHAP);
                            sBalans.setPosition(SBAAP);
                            _cba = true;
                        }
                        if (et.seconds() > (tims.get(timt) + CD + CC + CB + WT) * DT && !_csw) { /// Move the claw to the waiting position
                            conversiePerverssa(SAW);
                            sClaw.setPosition(SINCHIS);
                            _csw = true;
                        }
                        if (et.seconds() > (tims.get(timt) + CD + CC + CB + WT + ED) * DT) { /// Reset everything to normal
                            sBalans.setPosition(SBAG);
                            sClaw.setPosition(SINCHIS);
                            sHeading.setPosition(SHG);
                            conversiePerverssa(SAW);
                            coneReady = true;
                            armHolding = false;
                            coneClaw = false;
                            toGet = false;
                            cget = false;
                            cput = false;
                            waiting = true;
                            timt = 0;
                            et.reset();
                            extA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            extA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            extB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            extB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            spe(false, 0.0); // Te uras adi pt asta "Robotu ar trebui sa traga mereu bratele in spate cand face transferu fuck you"
                            //spe(true, 0.0); // Te uras adi pt asta "Robotu ar trebui sa traga mereu bratele in spate cand face transferu fuck you"
                            epd.curRet = false;
                        }
                    }
                }
            }

            if (!coneClaw && !cprepCone && !toPrepCone && toPut) { // If you want to put but do not have a cone in your claw, try to get one
                tppc = true;
                toPrepCone = true;
                cget = false;
            } else if (epd.target > MIP && !cprepCone && !toPrepCone && toPut) { // If you want to put but are too far extended, retract
                sHeading.setPosition(SHP);
                sBalans.setPosition(SBAP);
                conversiePerverssa(SAP + poff.get(2));
                ext(EMIN);
            }


            if (toPrepCone) { // Tries to grab the cone and set the robot in the holding state
                opcl();
                if (toPut) {
                    sClaw.setPosition(SINCHIS);
                }
                toPrepCone = false;
                cprepCone = true;
                cget = false;
                cput = false;
                rtg = false;
                if (BBBBBBBBBBBBB) {
                    DT = UPTC;
                } else if (tppc) {
                    DT = PUTC;
                } else {
                    DT = PREPC;
                }
                ct.reset();
            }

            if (!AAAAAAAAAAAAA) {
                if (cprepCone && ct.seconds() > CIP && ((epsEq(sextA.getPosition(), SAG) || epsEq(sextA.getPosition(), SAH) || epsEq(sextA.getPosition(), SAW)))) { /// Currently getting the cone
                    cprepCone = false;
                    if (!toPut) {
                        conversiePerverssa(SAH);
                        sBalans.setPosition(SBAH);
                    }
                    armHolding = true;
                    coneClaw = true;
                    cget = false;
                    if (!toPut || BBBBBBBBBBBBB) {
                        timt = 2;
                        BBBBBBBBBBBBB = false;
                    } else {
                        timt = 0;
                    }
                    if (epd.target > MIP && toPut) { /// Shortcut: to save on time when putting a cone while extended, start retracting immediatly
                        sHeading.setPosition(SHP);
                        sBalans.setPosition(SBAP);
                        conversiePerverssa(SAP + poff.get(2));
                        ext(EMIN);
                        timt = 1;
                    }
                } else if (cprepCone && ct.seconds() > CIP) {
                    TelemetryPacket tp = new TelemetryPacket();
                    tp.put("AAAAAAAAAAAAAAAAAAAAAAAAAAA+1", sextA.getPosition());
                    tp.put("AAAAAAAAAAAAAAAAAAAAAAAAAAA+2", SGS);
                    tp.put("AAAAAAAAAAAAAAAAAAAAAAAAAAA+3", epsEq(sextA.getPosition(), SGS));
                    dashboard.sendTelemetryPacket(tp);
                    if (epsEq(sextA.getPosition(), SGS)) {
                        sBalans.setPosition(SBAH);
                    } else {
                        sBalans.setPosition(SBAC);
                    }
                    conversiePerverssa(SAH);
                    AAAAAAAAAAAAA = true;
                    ct.reset();
                }
            } else if (ct.seconds() > 0.13) {
                cprepCone = false;
                timt = 2;
                AAAAAAAAAAAAA = false;
                BBBBBBBBBBBBB = true;
                toPrepCone = true;
            }

            if (toGet) { /// Setting the robot in the getting position
                tppc = false;
                toPut = false;
                cput = false;
                coneClaw = false;
                armHolding = false;
                toGet = false;
                cget = true;
                if (!((sextA.getPosition() > SAG + 0.01) && (sextA.getPosition() < SAH - 0.01))) {
                    conversiePerverssa(SAG);
                    sBalans.setPosition(SBAG);
                }
                opcl();
                rtg = true;
                gtim.reset();
            }

            if (!_clw && clw.seconds() > CLWT) {
                _clw = true;
                sClose.setPosition(SDESCHIS);
            }

            if (cget && gtim.seconds() > GHT * DT) { /// Currently Getting
                sHeading.setPosition(SHG);
                cget = false;
            }

            try {
                Thread.sleep(10); /// Avoid burning cycles, apparently this is not necessary on some CHubs
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
