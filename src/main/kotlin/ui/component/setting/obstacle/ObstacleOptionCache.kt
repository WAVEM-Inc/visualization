package ui.component.setting.obstacle

import application.config.ObstacleOption
import viewmodel.config.ObstacleViewModel

object ObstacleOptionCache {
    var obstacleOption = ObstacleOption()

    fun updateData() {
        val option = ObstacleViewModel.getObstacleOption()
        obstacleOption = option.copy()
    }

    fun saveAndApply() {
        ObstacleViewModel.saveAndApplyObstacleOption(obstacleOption)
    }
}