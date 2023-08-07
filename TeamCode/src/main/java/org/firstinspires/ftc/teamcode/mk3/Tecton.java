package org.firstinspires.ftc.teamcode.mk3;

import static org.firstinspires.ftc.teamcode.RobotVars.RMID_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.UPT;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.clo;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.rpd;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.wtfor_nonblocking;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Config
class Tecton implements Runnable {
    public LinearOpMode lom;
    public boolean shouldClose = false;
    Teotonom cav;

    public Tecton(Teotonom av) {
        cav = av;
    }

    int stage = 0;

    public static double W1 = 0.2;

    public void run() {
        ElapsedTime s3 = new ElapsedTime();
        while (!shouldClose && !lom.isStopRequested() && lom.opModeIsActive()) {
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
                    rpd.set_target(RMID_POS, UPT);
                    stage = 0;
                    break;
                default:
                    break;
            }
        }
    }
}