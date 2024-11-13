package frc.robot.subsystems.shooter

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.*
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.motors.TalonFXSim

class ShooterIOSim : ShooterIO {
    override val topRollerInputs = LoggedRollerInputs()
    override val bottomRollerInputs = LoggedRollerInputs()
    private val topMotor = TalonFXSim(
        1,
        GEAR_RATIO_TOP,
        MOMENT_OF_INERTIA_TOP.`in`(Units.KilogramSquareMeters),
        1.0
    )

    private val bottomMotor = TalonFXSim(
        1,
        GEAR_RATIO_BOTTOM,
        MOMENT_OF_INERTIA_BOTTOM.`in`(
            Units.KilogramSquareMeters
        ),
        1.0
    )

    private val topControl = VelocityVoltage(0.0)
    private val bottomControl = VelocityVoltage(0.0)
    private val stop = DutyCycleOut(0.0)

    init {
        topMotor.setController(
            PIDController(
                TOP_GAINS.kP,
                TOP_GAINS.kI,
                TOP_GAINS.kD
            )
        )
        bottomMotor.setController(
            PIDController(
                BOTTOM_GAINS.kP,
                BOTTOM_GAINS.kI,
                BOTTOM_GAINS.kD
            )
        )
    }

    override fun setTopVelocity(velocity: AngularVelocity) {
        topMotor.setControl(topControl.withVelocity(velocity.`in`(Units.RotationsPerSecond)))
    }

    override fun setBottomVelocity(velocity: AngularVelocity) {
        bottomMotor.setControl(bottomControl.withVelocity(velocity.`in`(Units.RotationsPerSecond)))
    }

    override fun stop() {
        bottomMotor.setControl(stop)
        topMotor.setControl(stop)
    }

    override fun setTopGains(kP: Double, kI: Double, kD: Double, kS: Double, kV: Double, kA: Double) {
        topMotor.setController(PIDController(kP, kI, kD))
    }

    override fun setBottomGains(kP: Double, kI: Double, kD: Double, kS: Double, kV: Double, kA: Double) {
        bottomMotor.setController(PIDController(kP, kI, kD))
    }

    override fun updateInputs() {
        topMotor.update(Timer.getFPGATimestamp())
        bottomMotor.update(Timer.getFPGATimestamp())

        topRollerInputs.velocity = Units.RotationsPerSecond.of(topMotor.velocity)
        bottomRollerInputs.velocity = Units.RotationsPerSecond.of(bottomMotor.velocity)
    }
}