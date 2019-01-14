package frc.team2767.motion.motion

import edu.wpi.first.wpilibj.Notifier
import mu.KotlinLogging
import org.strykeforce.thirdcoast.swerve.SwerveDrive
import org.strykeforce.thirdcoast.trapper.Action
import org.strykeforce.thirdcoast.trapper.Session
import org.strykeforce.thirdcoast.trapper.post

const val K_P = -1.2
const val GOOD_ENOUGH = 5500

private const val DT_MS = 20
private const val T1_MS = 200
private const val T2_MS = 100
private const val V_PROG = (12000 * 10).toDouble() // ticks/sec

class MotionController(private val drive: SwerveDrive, direction: Double, val distance: Int, azimuth: Double = 0.0) {
    private val logger = KotlinLogging.logger {}

    private val motionProfile = MotionProfile(DT_MS, T1_MS, T2_MS, V_PROG, distance)
    private val notifier = Notifier(this::updateDrive)

    private val ticksPerSecMax = drive.wheels[0].driveSetpointMax * 10.0
    private val forwardComponent = Math.cos(Math.toRadians(direction)) / ticksPerSecMax
    private val strafeComponent = Math.sin(Math.toRadians(direction)) / ticksPerSecMax

    private val start = IntArray(4)

    private var action =
        Action(
            name = "Skippy Motion Profile",
            measures = listOf("profile_ticks", "actual_ticks", "actual_distance"),
            traceMeasures = listOf(
                // "millis", are added as first data element but are not a measure
                "profile_acc",
                "profile_vel",
                "setpoint_vel",
                "actual_vel",
                "profile_ticks",
                "actual_ticks",
                "forward",
                "strafe",
                "yaw",
                "gyro_angle"
            )
        )


    init {
        drive.setDriveMode(SwerveDrive.DriveMode.CLOSED_LOOP)
        logger.debug {
            "INIT motion, direction = $direction, distance = $distance,\n" +
                    "ticks/sec max = $ticksPerSecMax\n" +
                    "forward = $forwardComponent, strafe = $strafeComponent\n"
        }

        action.meta["dt"] = DT_MS
        action.meta["t1"] = T1_MS
        action.meta["t2"] = T2_MS
        action.meta["vProg"] = V_PROG.toInt()
        action.meta["direction"] = direction
        action.meta["azimuth"] = azimuth
        action.meta["tags"] = listOf("skippy")
        action.meta["type"] = "motion_profile"
        action.meta["k_p"] = K_P
        action.meta["good_enough"] = GOOD_ENOUGH

        action.data.add(distance.toDouble()) // profile_ticks
        Session.baseUrl = "https://keeper.strykeforce.org"
    }

    val isFinished
        get() = motionProfile.isFinished && Math.abs(distance - actualDistance) < GOOD_ENOUGH

    private val actualDistance: Double
        get() {
            var distance = 0.0
            for (i in 0..3) distance += Math.abs(drive.wheels[i].driveTalon.getSelectedSensorPosition(0) - start[i])
            return distance / 4.0
        }

    private val actualVelocity: Int
        get() = drive.wheels[0].driveTalon.getSelectedSensorVelocity(0)

    fun start() {
        notifier.startPeriodic(DT_MS / 1000.0)
        logger.info("START motion, gyro angle = {}", drive.gyro.angle)
        action.meta["gyroStart"] = drive.gyro.angle.toString()

        for (i in 0..3) start[i] = drive.wheels[i].driveTalon.getSelectedSensorPosition(0)
    }

    fun stop() {
        notifier.stop()
        drive.drive(0.0, 0.0, 0.0)
        logger.info("FINISH motion position = {}", motionProfile.currPos)
        action.meta["gyroEnd"] = drive.gyro.angle
        action.data.add(actualDistance) // actual_ticks
        action.data.add(0.0) // actual_distance, measured physically

        action.post()
    }

    private fun updateDrive() {
        motionProfile.calculate()
        val velocity = motionProfile.currVel + K_P * positionError()
        val forward = forwardComponent * velocity
        val strafe = strafeComponent * velocity
        val yaw = 0.0
        drive.drive(forward, strafe, yaw)
        action.traceData.add(
            listOf(
                (motionProfile.iteration * DT_MS).toDouble(), // millis
                motionProfile.currAcc,     // profile_acc
                motionProfile.currVel,     // profile_vel
                velocity,                  // setpoint_vel
                actualVelocity.toDouble(), // actual_vel
                motionProfile.currPos,     // profile_ticks
                actualDistance,            // actual_ticks
                forward,  // forward
                strafe,   // strafe
                yaw,   // yaw
                drive.gyro.angle
            )
        )
    }

    private fun positionError() = actualDistance - motionProfile.currPos
}
