package ui.component.setting.lidar

import application.config.PointCloudOption
import com.google.gson.Gson
import util.common.LidarCalibration
import viewmodel.config.PointCloudViewModel

object LidarOption {
    private val gson = Gson()
    
    var pointCloudOption = PointCloudOption()
    var left: String = gson.toJson(LidarCalibration())
    var front: String = gson.toJson(LidarCalibration())
    var right: String = gson.toJson(LidarCalibration())

    fun updateData() {
        val option = PointCloudViewModel.getPointCloudOption()
        pointCloudOption = option.copy()
    }

    fun saveAndApply() {
        PointCloudViewModel.saveAndApplyPointCloudOption(pointCloudOption)
    }
}