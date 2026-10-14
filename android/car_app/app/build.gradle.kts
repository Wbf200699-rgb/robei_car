import java.util.Properties

plugins {
    alias(libs.plugins.android.application)
}

val localProperties = Properties().apply {
    val file = rootProject.file("local.properties")
    if (file.exists()) file.inputStream().use(::load)
}

fun readEnvValue(file: File, key: String): String? {
    if (!file.exists()) return null
    return file.useLines { lines ->
        lines.map(String::trim)
            .firstOrNull { it.startsWith("$key=") }
            ?.substringAfter('=')
            ?.trim()
            ?.trim('"', '\'')
    }
}

fun quotedBuildConfig(value: String): String =
    "\"" + value.replace("\\", "\\\\").replace("\"", "\\\"") + "\""

val uwbServerUrl = localProperties.getProperty("uwb.serverUrl")
    ?: "wss://relay.example.invalid/ws/mobile"
val uwbMobileToken = localProperties.getProperty("uwb.mobileToken")
    ?: System.getenv("UWB_MOBILE_TOKEN")
    ?: readEnvValue(rootProject.file("../../server/.env"), "UWB_MOBILE_TOKEN")
    ?: ""

android {
    namespace = "car.car_app"
    compileSdk {
        version = release(36)
    }

    defaultConfig {
        applicationId = "car.car_app"
        minSdk = 24
        targetSdk = 36
        versionCode = 2
        versionName = "1.1"

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
        buildConfigField("String", "UWB_SERVER_URL", quotedBuildConfig(uwbServerUrl))
        buildConfigField("String", "UWB_MOBILE_TOKEN", quotedBuildConfig(uwbMobileToken))
    }

    buildTypes {
        debug {
            applicationIdSuffix = ".debug"
            versionNameSuffix = "-debug"
        }
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }
    buildFeatures {
        buildConfig = true
    }
}

dependencies {
    implementation(libs.appcompat)
    implementation(libs.material)
    implementation(libs.activity)
    implementation(libs.constraintlayout)
    implementation(libs.okhttp)
    testImplementation(libs.junit)
    androidTestImplementation(libs.ext.junit)
    androidTestImplementation(libs.espresso.core)
}
