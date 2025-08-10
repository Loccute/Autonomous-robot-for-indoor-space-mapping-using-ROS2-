plugins {
    id("com.android.application")
    // START: FlutterFire Configuration
    id("com.google.gms.google-services")
    // END: FlutterFire Configuration
    id("kotlin-android")
    // The Flutter Gradle Plugin must be applied after the Android and Kotlin Gradle plugins.
    id("dev.flutter.flutter-gradle-plugin")
}

android {
    namespace = "com.example.test_mqtt_client"
    compileSdk = flutter.compileSdkVersion
    ndkVersion = flutter.ndkVersion

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    kotlinOptions {
        jvmTarget = "17"
    }

    defaultConfig {
        // TODO: Specify your own unique Application ID (https://developer.android.com/studio/build/application-id.html).
        applicationId = "com.example.test_mqtt_client"
        // You can update the following values to match your application needs.
        // For more information, see: https://flutter.dev/to/review-gradle-config.
        minSdk = flutter.minSdkVersion
        targetSdk = flutter.targetSdkVersion
        versionCode = flutter.versionCode
        versionName = flutter.versionName
    }

    buildTypes {
        release {
            // TODO: Add your own signing config for the release build.
            // Signing with the debug keys for now, so `flutter run --release` works.
            signingConfig = signingConfigs.getByName("debug")
        }
    }
}

flutter {
    source = "../.."
}

dependencies {
    // Kotlin stdlib
    implementation("org.jetbrains.kotlin:kotlin-stdlib:1.9.10")

    // Firebase Realtime Database KTX
    implementation("com.google.firebase:firebase-database-ktx:20.3.0")

    // Android core (cho Build, Intent, Notification, etc.)
    implementation("androidx.core:core-ktx:1.12.0")

    // AppCompat (để đảm bảo tương thích với các thành phần Android cũ hơn)
    implementation("androidx.appcompat:appcompat:1.6.1")

    // Lifecycle (nếu cần xử lý background service nâng cao)
    implementation("androidx.lifecycle:lifecycle-runtime-ktx:2.6.2")

    // Activity & service dependencies
    implementation("androidx.activity:activity-ktx:1.8.2")
}


