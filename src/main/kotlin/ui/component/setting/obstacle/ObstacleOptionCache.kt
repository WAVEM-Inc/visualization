package ui.component.setting.obstacle

import application.type.option.ObstacleOption
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import viewmodel.ConfigDataViewModel

object ObstacleOptionCache {
    var obstacleOption = ObstacleOption()

    fun updateData() {
        CoroutineScope(Dispatchers.Main).launch {
            val option = ConfigDataViewModel.getObstacleOption()
            if (option != null) {
                obstacleOption = option.copy()
            }
        }
    }

    fun saveAndApply() {
        CoroutineScope(Dispatchers.Main).launch {
            ConfigDataViewModel.saveAndApplyObstacleOption(obstacleOption)
        }
    }
}