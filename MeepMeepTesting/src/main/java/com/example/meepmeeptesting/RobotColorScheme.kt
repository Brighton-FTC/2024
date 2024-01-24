package com.example.meepmeeptesting

import com.noahbres.meepmeep.core.colorscheme.ColorManager
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark
import java.awt.Color

class RobotColorScheme : ColorSchemeRedDark() {
    override val TRAJECTORY_SLIDER_BG: Color = ColorManager.COLOR_PALETTE.GRAY_600
    override val TRAJECTORY_SLIDER_FG: Color = ColorManager.COLOR_PALETTE.GRAY_200
    override val TRAJECTORY_TEXT_COLOR: Color = ColorManager.COLOR_PALETTE.WHITE

    override val BOT_BODY_COLOR: Color = ColorManager.COLOR_PALETTE.GRAY_300
    override val BOT_DIRECTION_COLOR: Color = ColorManager.COLOR_PALETTE.GRAY_900
    override val BOT_WHEEL_COLOR: Color = ColorManager.COLOR_PALETTE.WHITE

    override val TRAJCETORY_PATH_COLOR: Color = ColorManager.COLOR_PALETTE.WHITE
    override val TRAJECTORY_MARKER_COLOR: Color = ColorManager.COLOR_PALETTE.RED_500
    override val TRAJECTORY_TURN_COLOR: Color = ColorManager.COLOR_PALETTE.RED_400
}