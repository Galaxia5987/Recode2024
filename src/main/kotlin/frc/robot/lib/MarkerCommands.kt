package frc.robot.lib

import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.WaitCommand

const val ABNORMAL_EVENT_NAME = "EVENT"

fun markEvent(eventName: String): Command = Commands.runOnce({ DataLogManager.log(eventName) })

fun markAbnormalEvent(): Command = markEvent(ABNORMAL_EVENT_NAME)