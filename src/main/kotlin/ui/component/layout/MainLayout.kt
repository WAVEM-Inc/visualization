package ui.component.layout

import androidx.compose.foundation.layout.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import ui.component.view.main.MainView

@Composable
fun MainLayout() {
    Row(modifier = Modifier.fillMaxSize().padding(10.dp)) {
        Box(modifier = Modifier.fillMaxSize(1f).weight(1f)) {
            MainView()
        }

        Spacer(modifier = Modifier.fillMaxHeight().width(10.dp))

        Column(modifier = Modifier.width(500.dp)) {
            SettingViewLayout()
        }
    }
}