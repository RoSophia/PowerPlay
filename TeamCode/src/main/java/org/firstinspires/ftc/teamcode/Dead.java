package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@SuppressWarnings("CommentedOutCode")
@Config
@Autonomous(group = "drive")
@SuppressLint("DefaultLocale")
public class Dead extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (!isStopRequested()) {

        }
    }
}
