package ui.component.setting.mobileye

import application.type.option.MobileyeOption
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import viewmodel.ConfigDataViewModel

object MobileyeOptionCache {
    var mobileyeOption = MobileyeOption()

    init {
        CoroutineScope(Dispatchers.Main).launch {
            ConfigDataViewModel.subscribeMobileyeOption(collector = { option ->
                MobileyeOptionCache.mobileyeOption = option.copy()
            })
        }
    }

    fun updateData() {
        CoroutineScope(Dispatchers.Main).launch {
            val option = ConfigDataViewModel.getMobileyeOption()
            if (option != null) {
                mobileyeOption = option.copy()
            }
        }
    }

    fun saveAndApply() {
        CoroutineScope(Dispatchers.Main).launch {
            ConfigDataViewModel.saveAndApplyMobileyeOption(mobileyeOption)
        }
    }
}