package frc.robot.subsystems.swerve

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.RobotConfig
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.Constants
import frc.robot.lib.controllers.DieterController
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import java.util.*
import java.util.function.DoubleSupplier
import java.util.function.Function
import kotlin.math.abs
import kotlin.math.hypot

class SwerveDrive private constructor
    (private val gyroIO: GyroIO, wheelOffsets: Array<Double>, vararg moduleIOs: ModuleIO) :
    SubsystemBase() {

    private val inputs = LoggedSwerveDriveInputs()

    private val modules = arrayOfNulls<SwerveModule>(moduleIOs.size) // FL, FR, RL, RR

    @AutoLogOutput
    private val modulePositions: Array<SwerveModulePosition?> = arrayOfNulls(4)

    @AutoLogOutput
    private var currentModuleStates: Array<SwerveModuleState?> = arrayOfNulls(modules.size)

    @AutoLogOutput
    private var desiredModuleStates: Array<SwerveModuleState?> = arrayOfNulls(modules.size)

    @AutoLogOutput
    private var chassisSpeeds = ChassisSpeeds()

    @AutoLogOutput
    private var desiredSpeeds = ChassisSpeeds()

    @AutoLogOutput
    private var turnAngleSetpoint: Measure<Angle> = Units.Degrees.zero()

    val atTurnSetpoint: Boolean
        get() = Units.Radians.of(yaw.rotations).isNear(turnAngleSetpoint, SwerveConstants.MAX_TURN_TOLERANCE)

    @AutoLogOutput
    var velocity = 0.0
        private set

    @AutoLogOutput
    private var absolutePositions = DoubleArray(4)

    private val kinematics = SwerveDriveKinematics(
        SwerveConstants.WHEEL_POSITIONS[0],
        SwerveConstants.WHEEL_POSITIONS[1],
        SwerveConstants.WHEEL_POSITIONS[2],
        SwerveConstants.WHEEL_POSITIONS[3]
    )

    val estimator: SwerveDrivePoseEstimator

    @AutoLogOutput
    private var botPose = Pose2d()

    private var inCharacterizationMode = false

    init {
        for (i in moduleIOs.indices) {
            modules[i] = SwerveModule(moduleIOs[i], i + 1, wheelOffsets[i])
        }
        for (i in modules.indices) {
            currentModuleStates[i] = SwerveModuleState()
            desiredModuleStates[i] = SwerveModuleState()
        }

        updateModulePositions()

        estimator =
            SwerveDrivePoseEstimator(
                kinematics, yaw, modulePositions, botPose
            )
    }

    companion object {
        @Volatile
        private var instance: SwerveDrive? = null

        fun initialize(gyroIO: GyroIO, offsets: Array<Double>, vararg moduleIOs: ModuleIO) {
            synchronized(this) {
                if (instance == null) {
                    instance = SwerveDrive(gyroIO, offsets, *moduleIOs)
                }
            }
        }

        fun getInstance(): SwerveDrive {
            return instance ?: throw IllegalStateException("Swerve has not been initialized. Call initialize first.")
        }
    }

    /**
     * Updates the offset for the gyro.
     *
     * @param angle The desired angle. [rad]
     */
    fun resetGyro(angle: Rotation2d = Rotation2d()) {
        gyroIO.resetGyro(angle)
    }

    val rawYaw
        /**
         * Gets the raw yaw reading from the gyro.
         *
         * @return Yaw angle reading from gyro. [rad]
         */
        get() = inputs.rawYaw

    val yaw
        /**
         * Gets the yaw reading from the gyro with the calculated offset.
         *
         * @return Yaw angle with offset. [rad]
         */
        get() = inputs.yaw

    val gyroYaw: Rotation2d
        get() {
            val alliance = DriverStation.getAlliance()
            if (alliance.isPresent && alliance.get() == DriverStation.Alliance.Red) {
                return yaw.minus(Rotation2d.fromDegrees(180.0))
            }
            return yaw
        }

    /**
     * Sets the module states to the desired module states.
     *
     * @param desiredModuleStates The desired module states to set the modules to.
     */
    fun setModuleStates(desiredModuleStates: Array<SwerveModuleState?>) {
        this.desiredModuleStates = desiredModuleStates
    }

    fun updateModulePositions() {
        for (i in modulePositions.indices) {
            modulePositions[i] = modules[i]?.modulePosition
        }
    }

    val currentSpeeds
        get() = chassisSpeeds

    fun resetPose(pose: Pose2d) {
        botPose = pose
        resetGyro(
            pose.rotation
                .minus(if (Constants.IS_RED) Rotation2d.fromDegrees(180.0) else Rotation2d())
        )
        estimator.resetPosition(pose.rotation, modulePositions, pose)
    }

    private fun isColliding() : Boolean {
        return abs(inputs.acceleration) > SwerveConstants.COLLISION_TOLERANCE.`in`(Units.Gs)
    }

    fun checkSwerve() {
        Arrays.stream(modules).forEach { obj: SwerveModule? -> obj!!.checkModule() }
    }

    fun stop() {
        for (i in modules.indices) {
            modules[i]?.moduleState = SwerveModuleState(0.0, modules[i]?.moduleState?.angle)
        }
    }

    fun lock() {
        desiredModuleStates =
            arrayOf<SwerveModuleState?>(
                SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
                SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)),
                SwerveModuleState(0.0, Rotation2d.fromDegrees(315.0)),
                SwerveModuleState(0.0, Rotation2d.fromDegrees(225.0))
            )
    }

    /**
     * Sets the correct module states from desired chassis speeds.
     *
     * @param chassisSpeeds Desired chassis speeds.
     * @param fieldOriented Should the drive be field oriented.
     */
    fun drive(chassisSpeeds: ChassisSpeeds, fieldOriented: Boolean) {
        var chassisSpeeds = chassisSpeeds
        desiredSpeeds = chassisSpeeds

        val fieldOrientedChassisSpeeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond,
                yaw
            )

        if (ChassisSpeeds(0.0, 0.0, 0.0) == chassisSpeeds) {
            Arrays.stream(modules).forEach { obj: SwerveModule? -> obj!!.stop() }
            return
        }

        if (fieldOriented) {
            chassisSpeeds = fieldOrientedChassisSpeeds
        }
        setModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds))
    }

    /**
     * Sets the desired percentage of x, y and omega speeds for the frc.robot.subsystems.swerve
     *
     * @param xOutput percentage of the max possible x speed
     * @param yOutput percentage of the max possible the y speed
     * @param omegaOutput percentage of the max possible rotation speed
     */
    fun drive(xOutput: Double, yOutput: Double, omegaOutput: Double, fieldOriented: Boolean) {
        val chassisSpeeds =
            ChassisSpeeds(
                SwerveConstants.MAX_X_Y_VELOCITY * xOutput,
                SwerveConstants.MAX_X_Y_VELOCITY * yOutput,
                SwerveConstants.MAX_OMEGA_VELOCITY * omegaOutput
            )

        drive(chassisSpeeds, fieldOriented)
    }

    fun driveCommand(
        forward: DoubleSupplier,
        strafe: DoubleSupplier,
        rotation: DoubleSupplier,
        deadband: Double = SwerveConstants.XBOX_DEADBAND,
        fieldOriented: Boolean = true
    ): Command {
        return run {
            drive(
                MathUtil.applyDeadband(forward.asDouble, deadband),
                MathUtil.applyDeadband(strafe.asDouble, deadband),
                MathUtil.applyDeadband(rotation.asDouble, deadband),
                fieldOriented
            )
        }
    }

    fun turnCommand(rotation: Measure<Angle>, turnTolerance: Double): Command {
        turnAngleSetpoint = rotation
        val turnController =
            DieterController(
                SwerveConstants.ROTATION_KP.get(),
                SwerveConstants.ROTATION_KI.get(),
                SwerveConstants.ROTATION_KD.get(),
                SwerveConstants.ROTATION_KDIETER.get()
            )
        turnController.setTolerance(turnTolerance)
        turnController.enableContinuousInput(-0.5, 0.5)
        return run {
            drive(
                0.0,
                0.0,
                turnController.calculate(
                    gyroYaw.rotations,
                    rotation.`in`(Units.Rotations)
                ),
                false
            )
        }
    }

    fun driveAndAdjust(
        rotation: ()->Measure<Angle>,
        forward: DoubleSupplier,
        strafe: DoubleSupplier,
        turnTolerance: Double,
        deadband: Double,
        usePoseEstimation: Boolean
    ): Command {
        turnAngleSetpoint = rotation.invoke()
        val turnController =
            DieterController(
                SwerveConstants.ROTATION_KP.get(),
                SwerveConstants.ROTATION_KI.get(),
                SwerveConstants.ROTATION_KD.get(),
                SwerveConstants.ROTATION_KDIETER.get()
            )
        turnController.enableContinuousInput(-0.5, 0.5)
        turnController.setTolerance(turnTolerance)
        return run {
            drive(
                MathUtil.applyDeadband(forward.asDouble, deadband),
                MathUtil.applyDeadband(strafe.asDouble, deadband),
                turnController.calculate(
                    if (usePoseEstimation
                    ) botPose.rotation.rotations
                    else gyroYaw.rotations,
                    rotation.invoke().`in`(Units.Rotations)
                ),
                true
            )
        }
    }

    fun updateSwerveOutputs() {
        currentModuleStates =
            Arrays.stream<SwerveModule?>(modules)
                .map<SwerveModuleState?>(Function<SwerveModule?, SwerveModuleState?> { obj: SwerveModule? -> obj?.moduleState })
                .toList()
                .toTypedArray<SwerveModuleState?>()
        kinematics
        chassisSpeeds = kinematics.toChassisSpeeds(*currentModuleStates)
        velocity = hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
        absolutePositions = Arrays.stream(modules)
            .mapToDouble { obj: SwerveModule? -> obj?.position ?: 0.0 } //TODO: really not sure about this on
            .toArray()
    }

    fun updateGyroInputs() {
        gyroIO.updateInputs(inputs)
    }

    val acceleration
        get() = inputs.acceleration

    override fun periodic() {
        updateSwerveOutputs()
        Arrays.stream(modules).forEach { obj: SwerveModule? -> obj!!.updateInputs() }
        updateGyroInputs()
        updateModulePositions()

        if (!isColliding()) {
            estimator.update(gyroYaw, modulePositions)
        }

        botPose = estimator.estimatedPosition

        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredModuleStates, SwerveConstants.MAX_X_Y_VELOCITY
        )
        if (!inCharacterizationMode) {
            for (i in modules.indices) {
                modules[i]?.moduleState = desiredModuleStates[i] ?: SwerveModuleState()
            }
        }

        Logger.processInputs("SwerveDrive", inputs)
        Logger.recordOutput("SwerveDrive/atTurnSetpoint", atTurnSetpoint)
        Logger.recordOutput("SwerveDrive/gyroYaw", gyroYaw)
        Logger.recordOutput("SwerveDrive/rawYaw", yaw)
    }

    fun configAutoBuilder() {
        AutoBuilder.configure(
            { botPose },
            this::resetPose,
            { chassisSpeeds },
            { speeds: ChassisSpeeds -> setModuleStates(kinematics.toSwerveModuleStates(speeds)) },
            SwerveConstants.DRIVE_CONTROLLER,
            RobotConfig.fromGUISettings(),
            { Constants.IS_RED },
            this
        )
    }

    fun characterize(): Command {
        val routine =
            SysIdRoutine(
                SysIdRoutine.Config(),
                SysIdRoutine.Mechanism(
                    { volts: Measure<Voltage?> ->
                        for (module in modules) {
                            module!!.characterize(
                                volts.`in`(edu.wpi.first.units.Units.Volts)
                            )
                        }
                    },
                    { log: SysIdRoutineLog ->
                        for (module in modules) {
                            module!!.updateSysIdRoutineLog(log)
                        }
                    },
                    this
                )
            )
        return Commands.sequence(
            Commands.runOnce({ inCharacterizationMode = true }),
            routine.dynamic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(1.0),
            routine.dynamic(SysIdRoutine.Direction.kReverse),
            Commands.waitSeconds(1.0),
            routine.quasistatic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(1.0),
            routine.quasistatic(SysIdRoutine.Direction.kReverse)
        )
            .finallyDo(Runnable { inCharacterizationMode = false })
    }
}
