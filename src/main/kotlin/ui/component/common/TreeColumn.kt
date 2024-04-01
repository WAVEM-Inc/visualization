package ui.component.common

import androidx.compose.foundation.layout.*
import androidx.compose.material.OutlinedTextField
import androidx.compose.material.Text
import androidx.compose.material.TextFieldDefaults
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.AnnotatedString
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontStyle
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.OffsetMapping
import androidx.compose.ui.text.input.TransformedText
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.TextUnit
import androidx.compose.ui.unit.TextUnitType
import androidx.compose.ui.unit.dp
import com.google.gson.Gson
import com.google.gson.JsonElement
import com.google.gson.JsonObject
import ui.theme.Neptune
import ui.theme.Poppins
import ui.theme.White_200

@Composable
fun <T : Any> TreeColumn(
    modifier: Modifier = Modifier,
    data: T,
    titleStyle: TextStyle = TextStyle(
        fontSize = TextUnit(16f, TextUnitType.Sp),
        fontFamily = Poppins,
        fontWeight = FontWeight.Bold,
        fontStyle = FontStyle.Normal,
        color = Color.Black
    ),
    headerStyle: TextStyle = TextStyle(
        fontSize = TextUnit(14f, TextUnitType.Sp),
        fontFamily = Poppins,
        fontWeight = FontWeight.SemiBold,
        fontStyle = FontStyle.Normal,
        color = Color.Black
    ),
    dataStyle: TextStyle = TextStyle(
        fontSize = TextUnit(12f, TextUnitType.Sp),
        fontFamily = Poppins,
        fontWeight = FontWeight.Normal,
        fontStyle = FontStyle.Normal,
        color = Color.Black,
        textAlign = TextAlign.Center
    ),
    title: String = data.javaClass.simpleName,
    onValueChange: (JsonObject) -> Unit
) {
    val jsonTree = Gson().toJsonTree(data)
    val jsonObject = jsonTree.asJsonObject
    val jsonMap = jsonObject.asMap()

    Column {
        Text(modifier = modifier, text = title, style = titleStyle)

        jsonMap.entries.forEach { e ->
            TreeItem(
                fieldName = e.key,
                element = e.value,
                headerStyle = headerStyle,
                dataStyle = dataStyle,
                jsonObject = jsonObject,
                onValueChange = { value ->
                    e.setValue(value)
                    onValueChange(jsonObject)
                }
            )
        }
    }
}

@Composable
fun <T : Any> TreeRow(
    modifier: Modifier = Modifier,
    data: T,
    titleStyle: TextStyle = TextStyle(
        fontSize = TextUnit(16f, TextUnitType.Sp),
        fontFamily = Poppins,
        fontWeight = FontWeight.Bold,
        fontStyle = FontStyle.Normal,
        color = Color.Black
    ),
    headerStyle: TextStyle = TextStyle(
        fontSize = TextUnit(14f, TextUnitType.Sp),
        fontFamily = Poppins,
        fontWeight = FontWeight.SemiBold,
        fontStyle = FontStyle.Normal,
        color = Color.Black
    ),
    dataStyle: TextStyle = TextStyle(
        fontSize = TextUnit(12f, TextUnitType.Sp),
        fontFamily = Poppins,
        fontWeight = FontWeight.Normal,
        fontStyle = FontStyle.Normal,
        color = Color.Black
    ),
    title: String = data.javaClass.simpleName,
    onValueChange: (JsonObject) -> Unit
) {
    val jsonTree = Gson().toJsonTree(data)
    val jsonObject = jsonTree.asJsonObject
    val jsonMap = jsonObject.asMap()

    Column(modifier = modifier) {
        Text(modifier = Modifier, text = title, style = titleStyle)

        Row {
            jsonMap.entries.forEach { e ->
                TreeItem(
                    fieldName = e.key,
                    element = e.value,
                    headerStyle = headerStyle,
                    dataStyle = dataStyle,
                    jsonObject = jsonObject,
                    onValueChange = { value ->
                        e.setValue(value)
                        onValueChange(jsonObject)
                    }
                )
            }
        }
    }
}

@Composable
private fun TreeItem(
    modifier: Modifier = Modifier,
    fieldName: String,
    headerStyle: TextStyle,
    dataStyle: TextStyle,
    element: JsonElement,
    jsonObject: JsonObject,
    onValueChange: (JsonObject) -> Unit
) {
    val tModifier = modifier.padding(start = 8.dp).width(100.dp).height(50.dp)

    Column(modifier = Modifier.padding(8.dp)) {
        if (element.isJsonObject) {
            TreeHeader(modifier = tModifier, fieldName = fieldName, headerStyle = headerStyle)

            element.asJsonObject.asMap().entries.forEach { entry ->
                TreeItem(
                    fieldName = entry.key,
                    element = entry.value,
                    headerStyle = headerStyle,
                    dataStyle = dataStyle,
                    jsonObject = element.asJsonObject,
                    onValueChange = onValueChange
                )
            }

        } else {
            TreeContents(
                modifier = tModifier,
                fieldName = fieldName,
                value = element,
                style = dataStyle,
                onValueChange = { s ->
                    jsonObject.addProperty(fieldName, s)
                    onValueChange(jsonObject)
                }
            )
        }
    }
}

@Composable
private fun TreeHeader(
    modifier: Modifier = Modifier,
    fieldName: String,
    headerStyle: TextStyle
) {
    Text(modifier = modifier, text = fieldName, style = headerStyle)
}

@Composable
private fun TreeContents(
    modifier: Modifier = Modifier,
    fieldName: String,
    style: TextStyle,
    value: JsonElement,
    onValueChange: (String) -> Unit
) {
    val tModifier = modifier.wrapContentSize()

    if (value.isJsonArray.not()) {
        var text by remember { mutableStateOf(value.asString) }
        Row(modifier = tModifier) {
            // Field Name
            Text(
                text = fieldName,
                style = style,
                modifier = Modifier.align(Alignment.CenterVertically).padding(end = 8.dp)
            )

            // Value of field
            OutlinedTextField(
                modifier = Modifier,
                value = text,
                textStyle = style,
                onValueChange = { s ->
                    text = s
                    onValueChange(s)
                },
                colors = TextFieldDefaults.textFieldColors(
                    focusedIndicatorColor = Neptune,
                    unfocusedIndicatorColor = White_200,
                    cursorColor = White_200
                )
            )
        }
    } else {
        // Field Name
        Row {
            Text(text = fieldName, style = style, modifier = Modifier.padding(end = 8.dp))
            Column(modifier = Modifier) {
                for (i in 0 until value.asJsonArray.size()) {
                    val element = value.asJsonArray[i]
                    var text by remember { mutableStateOf(element.asString) }

                    OutlinedTextField(
                        modifier = Modifier,
                        value = text,
                        textStyle = style,
                        onValueChange = { s ->
                            text = s
                            value.asJsonArray.set(i, Gson().toJsonTree(s))
                        },
                        visualTransformation = {
                            TransformedText(
                                text = AnnotatedString(""),
                                offsetMapping = OffsetMapping.Identity
                            )
                        },
                        colors = TextFieldDefaults.textFieldColors(
                            focusedIndicatorColor = Neptune,
                            unfocusedIndicatorColor = White_200,
                            cursorColor = White_200
                        )
                    )
                }
            }
        }
    }
}
