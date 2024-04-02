package ui.component.setting.lidar

import application.type.data.LidarCalibration
import application.type.option.PointCloudOption
import com.google.gson.Gson
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import viewmodel.ConfigDataViewModel


object LidarOption {
    private val gson = Gson()
    
    var pointCloudOption = PointCloudOption()
    var left: String = gson.toJson(LidarCalibration())
    var front: String = gson.toJson(LidarCalibration())
    var right: String = gson.toJson(LidarCalibration())

    init {
        CoroutineScope(Dispatchers.Main).launch {
            ConfigDataViewModel.subscribePointCloudOption(collector = {option ->
                pointCloudOption = option.copy()
            })
        }
    }

    fun updateData() {
        CoroutineScope(Dispatchers.Main).launch {
            val option = ConfigDataViewModel.getPointCloudOption()
            if (option != null) {
                pointCloudOption = option.copy()
            }
        }
    }

    fun saveAndApply() {
        CoroutineScope(Dispatchers.Main).launch {
            ConfigDataViewModel.saveAndApplyPointCloudOption(pointCloudOption)
        }
    }
}