// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.lib.PoseEstimation
import frc.robot.subsystems.vision.VisionConstants
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.FRCNetComm.tInstances
import frc.robot.subsystems.swerve.SwerveDrive
import org.littletonrobotics.junction.AutoLogOutput

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
object Robot : LoggedRobot() {
    private val compressor = Compressor(PneumaticsModuleType.CTREPCM)
    private var robotContainer: RobotContainer? = null
    private var autonomousCommand: Command? = null

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    override fun robotInit() {
        // Report Kotlin language usage
        // https://www.chiefdelphi.com/t/do-you-use-kotlin-make-sure-first-knows/447155?u=dan
        HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin)
        
        initializeSubsystems(Constants.CURRENT_MODE)

        // Initialize logger
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME)
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE)
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA)
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE)
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH)
        when (BuildConstants.DIRTY) {
            0 -> Logger.recordMetadata("GitDirty", "All changes committed")
            1 -> Logger.recordMetadata("GitDirty", "Uncomitted changes")
            else -> Logger.recordMetadata("GitDirty", "Unknown")
        }
        when (Constants.CURRENT_MODE) {
            Constants.Mode.REAL -> {
                LoggedPowerDistribution.getInstance(0, PowerDistribution.ModuleType.kCTRE)
                Logger.addDataReceiver(WPILOGWriter())
                Logger.addDataReceiver(NT4Publisher())
            }

            Constants.Mode.SIM -> Logger.addDataReceiver(NT4Publisher())
            Constants.Mode.REPLAY -> {
                setUseTiming(false)
                val logPath = LogFileUtil.findReplayLog()
                Logger.setReplaySource(WPILOGReader(logPath))
                Logger.addDataReceiver(
                        WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")))
            }
        }
        Logger.start()
        SignalLogger.enableAutoLogging(true)

        robotContainer = RobotContainer
        compressor.enableDigital()

        DriverStation.silenceJoystickConnectionWarning(true)
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     *
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    override fun robotPeriodic() {
        PoseEstimation.getInstance().processVisionMeasurements(VisionConstants.VISION_MEASUREMENT_MULTIPLIER)
        CommandScheduler.getInstance().run()
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select between different
     * autonomous modes using the dashboard. The sendable chooser code works with the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
     * uncomment the getString line to get the auto name from the text box below the Gyro
     *
     *
     * You can add additional auto modes by adding additional comparisons to the switch structure
     * below with additional strings. If using the SendableChooser make sure to add them to the
     * chooser code above as well.
     */
    override fun autonomousInit() {
        // Make sure command is compiled beforehand, otherwise there will be a delay.
        autonomousCommand = robotContainer!!.getAutonomousCommand()

        // Schedule the autonomous command
        autonomousCommand!!.schedule()
    }

    /** This function is called periodically during autonomous.  */
    override fun autonomousPeriodic() {

    }

    /** This function is called once when teleop is enabled.  */
    override fun teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand!!.cancel()
        }
    }

    /** This function is called periodically during operator control.  */
    override fun teleopPeriodic() {}

    /** This function is called once when the robot is disabled.  */
    override fun disabledInit() {}

    /** This function is called periodically when disabled.  */
    override fun disabledPeriodic() {}

    /** This function is called once when test mode is enabled.  */
    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    /** This function is called periodically during test mode.  */
    override fun testPeriodic() {}

    @AutoLogOutput
    fun getDistanceToSpeaker(): Double = (
            Constants.SPEAKER_POSE - SwerveDrive.getInstance().estimator.estimatedPosition.translation
            ).norm

}
