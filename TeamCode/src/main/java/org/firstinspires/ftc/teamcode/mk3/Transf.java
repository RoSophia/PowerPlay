package org.firstinspires.ftc.teamcode.mk3;

import static org.firstinspires.ftc.teamcode.RobotVars.SAG;
import static org.firstinspires.ftc.teamcode.RobotVars.SAP;
import static org.firstinspires.ftc.teamcode.RobotVars.SBG;
import static org.firstinspires.ftc.teamcode.RobotVars.SBP;
import static org.firstinspires.ftc.teamcode.RobotVars.SCC;
import static org.firstinspires.ftc.teamcode.RobotVars.SCO;
import static org.firstinspires.ftc.teamcode.RobotVars.SDESCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.SHG;
import static org.firstinspires.ftc.teamcode.RobotVars.SHP;
import static org.firstinspires.ftc.teamcode.RobotVars.SINCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.USE_TELE;
import static org.firstinspires.ftc.teamcode.RobotVars.coneReady;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@SuppressWarnings("ALL")
@Config
public class Transf implements Runnable {
    public static int MIP = 100;
    public static double ME = 0.01;
    public static double ETC = 0.3;

    private Servo sa, sb, sh, sg, sMClaw;
    private DcMotorEx ce;
    private boolean cput = false;
    public boolean toPut = false;
    public boolean toGet = false;
    public boolean shouldClose = true;

    boolean toOpenSC;
    ElapsedTime et = new ElapsedTime(0);

    public Transf(Servo sa, Servo sb, Servo sh, Servo sg, Servo sMClaw, DcMotorEx ce) {
        this.sa = sa;
        sa.setPosition(SAG);
        this.sb = sb;
        sb.setPosition(SBG);
        this.sh = sh;
        sh.setPosition(SHG);
        this.sg = sg;
        sg.setPosition(SDESCHIS);
        this.sMClaw = sMClaw;
        sMClaw.setPosition(SCO);
        this.ce = ce;
    }

    public void run() {
        while (shouldClose) {
            if (USE_TELE) {
                TelemetryPacket packet = new TelemetryPacket();

                packet.put("toPut", toPut);
                packet.put("cput", cput);
                packet.put("sa", (sa.getPosition() - SAP) / (SAP - SAG));
                packet.put("sb", (sb.getPosition() - SBP) / (SBP - SBG));
                packet.put("sh", (sh.getPosition() - SHP) / (SHP - SHG));
                packet.put("sg", (sg.getPosition() - SINCHIS) / (SINCHIS - SDESCHIS));
                packet.put("sc", (sMClaw.getPosition() - SCC) / (SCC - SCO));

                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }

            if (cput && toPut) {
                toPut = false;
            }

            if (toPut && ce.getCurrentPosition() < MIP) {
                toPut = false;
                cput = true;
                sa.setPosition(SAP);
                sb.setPosition(SBP);
                sh.setPosition(SHP);
                sg.setPosition(SINCHIS);
                sMClaw.setPosition(SCC);
            }

            if (cput) {
                if (Math.abs(sa.getPosition() - SAP) > ME) {
                    sg.setPosition(SDESCHIS);
                    toOpenSC = true;
                    et.reset();
                    cput = false;
                }
            }

            if (toOpenSC && et.seconds() > ETC) {
                sMClaw.setPosition(SCO);
                coneReady = true;
                toOpenSC = false;
            }

            if (toGet) {
                toGet = false;
                toPut = false;
                cput = false;
                sa.setPosition(SAG);
                sb.setPosition(SBG);
                sh.setPosition(SHG);
                sg.setPosition(SDESCHIS);
                sMClaw.setPosition(SCO);
            }
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
