package frc.robot.subsystems.intake

import edu.wpi.first.units.Units
import frc.robot.subsystems.BaseSubsystemTest
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

class IntakeTest : BaseSubsystemTest() {
    private lateinit var intake: Intake

    @BeforeEach
    fun initializeIntake() {
        val intakeIO = IntakeIOSim()
        Intake.initialize(intakeIO)
        intake = Intake.getInstance()
    }

    @Test
    fun testSetSpinPower() {
        intake.setSpinPower(4.0).execute()

        simulateTimeAndWait(5.0, 1000)

        assertTrue(intake.getInputs().spinMotorVoltage != 0.0, "No voltage to spin motor")
    }

    @Test
    fun testSetCenterPower() {
        intake.setCenterPower(4.0).execute()

        simulateTimeAndWait()

        assertTrue(intake.getInputs().centerMotorVoltage != 0.0, "No voltage to center motor")
    }

    @Test
    fun testSetAngle() {
        val angle = Units.Degrees.of(30.0)

        intake.setAngle(angle).initialize() // setAngle is runOnce so the action is in the init

        simulateTimeAndWait()

        val isAtSetpoint =
            intake.getInputs().angleMotorAngle.isNear(angle, 0.5)

        assertTrue(isAtSetpoint)
    }
}