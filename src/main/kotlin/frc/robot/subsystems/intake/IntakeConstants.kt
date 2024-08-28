package frc.robot.subsystems.intake

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import frc.robot.Constants
import frc.robot.lib.webconstants.LoggedTunableNumber

object IntakeConstants {
    const val INTAKE_SPIN_POWER = -0.4
    const val INTAKE_CENTER_POWER = -0.4
    const val GEAR_RATIO = 55.56

    val ANGLE_KP = LoggedTunableNumber("Intake/Angle/kP")
    val ANGLE_KI = LoggedTunableNumber("Intake/Angle/kI")
    val ANGLE_KD = LoggedTunableNumber("Intake/Angle/kD")
    val ANGLE_KG = LoggedTunableNumber("Intake/Angle/kG")
    val PID_VALUES = arrayOf(ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_KG)

    val MOTOR_CONFIG = TalonFXConfiguration()

    val INTAKE_ANGLE: Measure<Angle> = Units.Degree.zero()
    val REST_ANGLE: Measure<Angle> = Units.Degree.of(120.0)

    init {
        when (Constants.CURRENT_MODE) {
            Constants.Mode.REAL -> {
                ANGLE_KP.initDefault(21.0)
                ANGLE_KI.initDefault(0.0)
                ANGLE_KD.initDefault(0.1)
                ANGLE_KG.initDefault(0.0)
            }

            Constants.Mode.SIM, Constants.Mode.REPLAY -> {
                ANGLE_KP.initDefault(10.1 / 360.0)
                ANGLE_KI.initDefault(0.0)
                ANGLE_KD.initDefault(0.0)
            }
        }

        MOTOR_CONFIG.withMotorOutput(
            MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive)
        ).withFeedback(
            FeedbackConfigs()
                .withRotorToSensorRatio(1.0)
                .withSensorToMechanismRatio(GEAR_RATIO)
        )
            .withSlot0(
            Slot0Configs()
                .withKP(ANGLE_KP.get())
                .withKI(ANGLE_KI.get())
                .withKD(ANGLE_KD.get())
                .withKG(ANGLE_KG.get())
        ).CurrentLimits
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80.0)
            .withSupplyCurrentLimit(40.0)
    }
}
