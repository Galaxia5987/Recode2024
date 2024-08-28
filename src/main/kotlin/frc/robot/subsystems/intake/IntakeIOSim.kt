package frc.robot.subsystems.intake

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import frc.robot.lib.motors.SparkMaxSim
import frc.robot.lib.motors.TalonFXSim
import edu.wpi.first.wpilibj.Timer

class IntakeIOSim : IntakeIO {
    override val inputs = LoggedIntakeInputs()

    private val angleMotor = TalonFXSim(1, IntakeConstants.GEAR_RATIO, 0.003, 360 * IntakeConstants.GEAR_RATIO)
    private val spinMotor = SparkMaxSim(1, 1.0, 0.003, 1.0)
    private val centerMotor = SparkMaxSim(1, 1.0, 0.003, 1.0)
    private val positionControl = PositionVoltage(0.0)
    private val dutyCycle = DutyCycleOut(0.0)
    private val angleController: PIDController =
        PIDController(IntakeConstants.ANGLE_KP.get(), IntakeConstants.ANGLE_KI.get(), IntakeConstants.ANGLE_KD.get())

    init {
        angleMotor.setController(angleController)
    }

    override fun setSpinPower(power: Double) {
        spinMotor.set(power)
    }

    override fun setCenterPower(power: Double) {
        centerMotor.set(power)
    }

    override fun setAngle(angle: Measure<Angle>) {
        angleMotor.setControl(positionControl.withPosition(angle.`in`(Units.Rotations)))
    }

    override fun setAnglePower(power: Double) {
        angleMotor.setControl(dutyCycle.withOutput(power))
    }

    override fun updateInputs() {
        angleMotor.update(Timer.getFPGATimestamp())
        spinMotor.update(Timer.getFPGATimestamp())
        centerMotor.update(Timer.getFPGATimestamp())
        inputs.spinMotorVoltage = spinMotor.busVoltage
        inputs.centerMotorVoltage = centerMotor.busVoltage
        inputs.angleMotorAngle = Units.Rotations.of(angleMotor.position)
    }

}