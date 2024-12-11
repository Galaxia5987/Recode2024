package frc.robot.subsystems.elevator

import frc.robot.lib.Gains
import frc.robot.lib.selectGainsBasedOnMode

const val MAX_HEIGHT = 1.3
const val GEAR_RATIO = 0.2
const val FIRSTSTAGE_GEARRATIOE = 0.5

const val FIRST_STAGE_RATIO = 0.5
val GAINS = selectGainsBasedOnMode(Gains(), Gains())