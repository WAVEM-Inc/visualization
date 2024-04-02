import org.jetbrains.compose.desktop.application.dsl.TargetFormat

plugins {
    kotlin("jvm")
    id("org.jetbrains.compose")
}

group = "net.ums"
version = "1.0-SNAPSHOT"

repositories {
    mavenCentral()
    maven("https://maven.pkg.jetbrains.space/public/p/compose/dev")
    google()
}

dependencies {
    // Note, if you develop a library, you should use compose.desktop.common.
    // compose.desktop.currentOs should be used in launcher-sourceSet
    // (in a separate module for demo project and in testMain).
    // With compose.desktop.common you will also lose @Preview functionality
    implementation(compose.desktop.currentOs)

    // Protobuf
    implementation("com.google.protobuf:protobuf-kotlin:4.26.0")

    // JOGL
    implementation("org.jogamp.jogl:jogl-all-main:2.3.2")
    implementation("org.joml:joml:1.10.5")
    implementation("org.jogamp.gluegen:gluegen-rt-main:2.3.2")



    // File Picker
    implementation("com.darkrockstudios:mpfilepicker:2.1.0")

    // Color Picker
    implementation("com.godaddy.android.colorpicker:compose-color-picker:0.7.0")
    implementation("com.godaddy.android.colorpicker:compose-color-picker-jvm:0.7.0")

    // Gson
    implementation("com.google.code.gson:gson:2.10.1")

    // Jackson
    implementation("com.fasterxml.jackson.core:jackson-databind:2.14.2")
    implementation("com.fasterxml.jackson.module:jackson-module-kotlin:2.14.2")
    implementation("com.fasterxml.jackson.dataformat:jackson-dataformat-yaml:2.14.2")

    // JXMapViewer
    implementation("org.jxmapviewer:jxmapviewer2:2.6")

    // ProJ4J
    implementation("org.locationtech.proj4j:proj4j:1.3.0")
}

compose.desktop {
    application {
        mainClass = "MainKt"

        nativeDistributions {
            targetFormats(TargetFormat.Dmg, TargetFormat.Msi, TargetFormat.Deb)
            packageName = "Eviz"
            packageVersion = "1.0.0"

            appResourcesRootDir.set(project.layout.projectDirectory.dir("resources"))
            outputBaseDir.set(project.buildDir.resolve("testOutputDir"))
        }
    }
}
