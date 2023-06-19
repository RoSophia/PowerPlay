package org.firstinspires.ftc.teamcode.mk3;

import static org.firstinspires.ftc.teamcode.RobotVars.RTOP_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.UPT;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.clo;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.rpd;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.wtfor_nonblocking;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
class FireExtenguisher implements Runnable {
    public LinearOpMode lom;
    public boolean shouldClose = false;
    FIRE cav;

    public FireExtenguisher(FIRE av) {
        cav = av;
    }

    int stage = 0;

    public static double W1 = 0.2;

    public void run() {
        ElapsedTime s3 = new ElapsedTime();
        TelemetryPacket tp;
        while (!shouldClose && !lom.isStopRequested() && lom.opModeIsActive()) {
            /*tp = new TelemetryPacket();
            tp.put("IHK_stage", stage);
            FtcDashboard.getInstance().sendTelemetryPacket(tp);*/
            switch (stage) {
                case 1:
                    clo.toPut = true;
                    stage = 2;
                    break;
                case 2:
                    if (wtfor_nonblocking(RobotFuncs.WAITS.TRANSFER)) {
                        stage = 3;
                        s3.reset();
                        cav.st_grab_pos();
                    }
                    break;
                case 3:
                    if (s3.seconds() >= W1) {
                        stage = 4;
                    }
                    break;
                case 4:
                    rpd.set_target(RTOP_POS, UPT);
                    stage = 0;
                    break;
                default:
                    break;
            }
        }
    }
}