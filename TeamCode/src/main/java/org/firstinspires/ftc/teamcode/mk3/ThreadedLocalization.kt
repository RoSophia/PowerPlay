package org.firstinspires.ftc.teamcode.mk3

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.sync.Mutex
import org.firstinspires.ftc.teamcode.RobotVars.*
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import java.lang.Thread.sleep
import kotlin.math.abs

class ThreadedLocalization(dr: SampleMecanumDrive) : Runnable {
    lateinit var lom: LinearOpMode
    val drive: SampleMecanumDrive
    var shouldClose = false

    private val mtx = Mutex()
    private var new = true
    private lateinit var poseEstimate: Pose2d
    private var poseVelocity: Pose2d? = null
    private var lastError: Pose2d? = null

    private fun updEst() {
        while (!mtx.tryLock()) {
            sleep(3)
        }
        poseEstimate = drive.poseEstimate
        poseVelocity = drive.poseVelocity
        lastError = drive.lastError
        val tp = TelemetryPacket()
        tp.put("EST_PEX", poseEstimate.x)
        tp.put("EST_PEY", poseEstimate.y)
        tp.put("EST_PEH", poseEstimate.heading)
        tp.put("EST_PVX", poseVelocity?.x)
        tp.put("EST_PVY", poseVelocity?.y)
        tp.put("EST_PVH", poseVelocity?.heading)
        tp.put("EST_LEX", lastError?.x)
        tp.put("EST_LEY", lastError?.y)
        tp.put("EST_LEH", lastError?.heading)
        FtcDashboard.getInstance().sendTelemetryPacket(tp)
        new = false
        mtx.unlock()
    }

    fun updHeading() {
        if (USE_UPD_HEAD && abs(RobotFuncs.imu.lastRead) > 0.001) {
            while (!mtx.tryLock()) {
                sleep(3)
            }
            val clr = RobotFuncs.imu.lastRead // Seramitae
            RobotFuncs.imu.updated = false
            val cpe = drive.poseEstimate
            drive.poseEstimate = Pose2d(cpe.x, cpe.y, clr)
            mtx.unlock()
        }
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

    fun getLastError(): Pose2d? {
        if (new) {
            updEst()
        }
        return lastError
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
            if (USE_UPD_HEAD_FULL) {
                updHeading()
            }
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