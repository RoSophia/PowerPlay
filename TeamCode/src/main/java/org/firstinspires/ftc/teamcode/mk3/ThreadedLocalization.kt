package org.firstinspires.ftc.teamcode.mk3

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.sync.Mutex
import org.firstinspires.ftc.teamcode.RobotVars.LOCALIZATION_SLEEP_TIME
import org.firstinspires.ftc.teamcode.RobotVars.USE_TELE
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import java.lang.Thread.sleep

class ThreadedLocalization(dr: SampleMecanumDrive): Runnable {
    lateinit var lom: LinearOpMode
    val drive: SampleMecanumDrive
    var shouldClose = false

    private val mtx = Mutex()
    private var new = true
    private lateinit var poseEstimate: Pose2d
    private var poseVelocity: Pose2d? = null

    private fun updEst()  {
        while (!mtx.tryLock()) {
            sleep(3)
        }
        poseEstimate = drive.poseEstimate
        poseVelocity = drive.poseVelocity
        new = false
        mtx.unlock()
    }

    fun getPoseEstimate(): Pose2d {
        if (new) {
            updEst()
        }
        return poseEstimate
    }

    fun getPoseVelocity(): Pose2d? {
        if (new) {
            updEst()
        }
        return poseVelocity
    }

    init {
        drive = dr
    }

    override fun run() {
        val ep = ElapsedTime()
        val pe = ElapsedTime()
        ep.reset()
        pe.reset()
        while (!shouldClose && !lom.isStopRequested && !Thread.interrupted()) {
            while (!mtx.tryLock()) {
                sleep(3)
            }
            pe.reset()
            drive.updatePoseEstimate()
            new = true
            mtx.unlock()
            if (USE_TELE) {
                val tp = TelemetryPacket()
                tp.put("UPD_POSE_EST_THREAD_TIME", pe.milliseconds())
                tp.put("UPD_POSE_EST_THREAD", ep.milliseconds())
                ep.reset()
                FtcDashboard.getInstance().sendTelemetryPacket(tp)
            }
            sleep(LOCALIZATION_SLEEP_TIME.toLong())
        }
    }
}