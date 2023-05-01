package org.firstinspires.ftc.teamcode.mk3;

import static org.firstinspires.ftc.teamcode.RobotVars.EMIN;
import static org.firstinspires.ftc.teamcode.RobotVars.RETT;
import static org.firstinspires.ftc.teamcode.RobotVars.SAG;
import static org.firstinspires.ftc.teamcode.RobotVars.SAH;
import static org.firstinspires.ftc.teamcode.RobotVars.SAP;
import static org.firstinspires.ftc.teamcode.RobotVars.SAW;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAG;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAH;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAP;
import static org.firstinspires.ftc.teamcode.RobotVars.SCC;
import static org.firstinspires.ftc.teamcode.RobotVars.SCO;
import static org.firstinspires.ftc.teamcode.RobotVars.SDESCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.SHG;
import static org.firstinspires.ftc.teamcode.RobotVars.SHP;
import static org.firstinspires.ftc.teamcode.RobotVars.SINCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.USE_TELE;
import static org.firstinspires.ftc.teamcode.RobotVars.armHolding;
import static org.firstinspires.ftc.teamcode.RobotVars.coneClaw;
import static org.firstinspires.ftc.teamcode.RobotVars.coneReady;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.conversiePerverssa;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.epd;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.ext;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;

@SuppressWarnings("ALL")
@Config
public class Clown implements Runnable {
    public static int MIP = 100;
    public static double ME = 5;
    public static double ETC = 0.14;
    public static double CPT = 0.45;
    public static double CET = 0.5;
    public static double CHT = 0.75;
    public static double TTT = 0.0;
    public static boolean CLAW = false;
    public static double PUTC = 1.212;
    public static double PREPC = 0.8;
    public static double CIP = 0.11;
    public static double GB = 0.20;
    public static double GHT = 0.1;
    public static double WT = 0.1;
    public static double CD = 0.0;
    public static double ED = 0.3;
    public static double GDI = -0.002;

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
    public double hdif = 0.000;

    public DistanceSensor csensor;
    ElapsedTime ht = new ElapsedTime(0);

    List<Double> tims = Arrays.asList(CPT, CET, CHT);
    int timt = 0;

    ElapsedTime et = new ElapsedTime(0);
    ElapsedTime ct = new ElapsedTime(0);

    public Clown(Servo sa, Servo sb, Servo sHeading, Servo sClaw, Servo sMClaw, Servo sBalans, DcMotorEx ce, DistanceSensor csensor) {
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

    public void run() {
        while (!shouldClose) {
            if (USE_TELE) {
                TelemetryPacket packet = new TelemetryPacket();
                tims = Arrays.asList(CPT, CET, CHT);

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

                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }

            /* 
             * This whole class works by the principle of saying what you want to do, 
             * and it trying to get the internal state of the robot to the appropriate one to do what you need.
             */

            /*
            if (csensor.distance() < CDIST && !coneClaw && !coneReady) {
                toPrepCone = true;
            }*/

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

                double cd = 0;
                if (timt == 0) {
                    cd = GDI;
                }
                conversiePerverssa(SAP + cd + hdif);
                ext(EMIN);
                //sHeading.setPosition(SHP);
                sClaw.setPosition(SINCHIS);
                sMClaw.setPosition(SCO);
                sBalans.setPosition(SBAP);
                et.reset();
            }

            if (cput) { // Currently putting
                if (et.seconds() > TTT * DT) { /// Start rotating the claw after TTT time to avoid bumping the cone into the whole arm
                    sHeading.setPosition(SHP);
                } else {
                    double cd = 0;
                    if (timt == 0) {
                        cd = GDI;
                    }
                    conversiePerverssa(SAP + cd + hdif);
                    sClaw.setPosition(SINCHIS);
                    sMClaw.setPosition(SCO);
                    sBalans.setPosition(SBAP);
                }

                if (et.seconds() > tims.get(timt) * DT) { /// Open the claw after it has reached the holding bay
                    sClaw.setPosition(SDESCHIS);
                }
                if (et.seconds() > (tims.get(timt) + WT) * DT) { /// Move the claw to the waiting position
                    conversiePerverssa(SAW);
                }
                if (et.seconds() > (tims.get(timt) + CD) * DT) { /// Close the mini servo to keep the cone in the holding bay
                    sMClaw.setPosition(SCC);
                }
                if (et.seconds() > (tims.get(timt) + ED) * DT) { /// Reset everything to normal
                    sBalans.setPosition(SBAG);
                    sClaw.setPosition(SDESCHIS);
                    sHeading.setPosition(SHG);
                    coneReady = true;
                    armHolding = false;
                    coneClaw = false;
                    toGet = false;
                    cget = false;
                    cput = false;
                    waiting = true;
                    timt = 0;
                    et.reset();
                }

            }

            if (!coneClaw && !cprepCone && !toPrepCone && toPut) { // If you want to put but do not have a cone in your claw, try to get one
                tppc = true;
                toPrepCone = true;
                cget = false;
            } else if (epd.target > MIP && !cprepCone && !toPrepCone && toPut) { // If you want to put but are too far extended, retract
                sHeading.setPosition(SHP);
                sBalans.setPosition(SBAP);
                conversiePerverssa(SAP);
                ext(EMIN);
            }


            if (toPrepCone) { // Tries to grab the cone and set the robot in the holding state
                if (toPut) {
                    sClaw.setPosition(SINCHIS);
                }
                toPrepCone = false;
                cprepCone = true;
                cget = false;
                rtg = false;
                if (tppc) {
                    DT = PUTC;
                } else {
                    DT = PREPC;
                }
                ct.reset();
            }

            if (cprepCone && ct.seconds() > CIP) { /// Currently getting the cone
                cprepCone = false;
                conversiePerverssa(SAH);
                sBalans.setPosition(SBAH);
                armHolding = true;
                coneClaw = true;
                cget = false;
                if (!toPut) {
                    timt = 2;
                } else{
                    timt = 0;
                }
                if (epd.target > MIP && toPut) { /// Shortcut: to save on time when putting a cone while extended, start retracting immediatly
                    sHeading.setPosition(SHP);
                    sBalans.setPosition(SBAP);
                    conversiePerverssa(SAP);
                    ext(EMIN);
                    timt = 1;
                }
            }

            if (toGet) { /// Setting the robot in the getting position
                tppc = false;
                toPut = false;
                cput = false;
                coneClaw = false;
                armHolding = false;
                toGet = false;
                cget = true;
                /*
                if (UST > 0) {
                    conversiePerverssa(ST - SD * (UST - 1));
                    sBalans.setPosition(SBT - SBD * (UST - 1));
                } else {*/
                    conversiePerverssa(SAG);
                    sBalans.setPosition(SBAG);
                //}
                rtg = true;
                gtim.reset();
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
