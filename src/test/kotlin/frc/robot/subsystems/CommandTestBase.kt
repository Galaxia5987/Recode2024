package frc.robot.subsystems

import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.simulation.DriverStationSim
import edu.wpi.first.wpilibj.simulation.SimHooks
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.parallel.ResourceLock

open class CommandTestBase {
    @BeforeEach
    fun commandSetup() {
        CommandScheduler.getInstance().cancelAll()
        CommandScheduler.getInstance().enable()
        CommandScheduler.getInstance().activeButtonLoop.clear()
        CommandScheduler.getInstance().clearComposedCommands()
        CommandScheduler.getInstance().unregisterAllSubsystems()

        setDSEnabled(true)
    }

    fun setDSEnabled(enabled: Boolean) {
        DriverStationSim.setDsAttached(true)
        DriverStationSim.setEnabled(enabled)
        DriverStationSim.notifyNewData()
        while (DriverStation.isEnabled() != enabled) {
            try {
                Thread.sleep(1)
            } catch (exception: InterruptedException) {
                exception.printStackTrace()
            }
        }
    }

    @ResourceLock("timing")
    fun testInstantCommand(command: Command, deltaSeconds: Double) {
        HAL.initialize(500, 0)
        SimHooks.pauseTiming()
        try {
            CommandScheduler.getInstance().use { scheduler: CommandScheduler ->
                scheduler.schedule(command)
                scheduler.run()
                SimHooks.stepTiming(deltaSeconds)
                scheduler.run()
                assertFalse(scheduler.isScheduled(command))
            }
        } finally {
            SimHooks.resumeTiming()
        }
    }
}
