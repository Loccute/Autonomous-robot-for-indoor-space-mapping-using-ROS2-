buildscript {
    repositories {
        google()
        mavenCentral()
    }
    dependencies {
        // Plugin cho Gradle Android
        classpath("com.android.tools.build:gradle:8.2.1")

        // Plugin cho Google Services (Firebase)
        classpath("com.google.gms:google-services:4.4.0")

        // Plugin Kotlin cần thiết
        classpath("org.jetbrains.kotlin:kotlin-gradle-plugin:1.9.10")
    }
}

allprojects {
    repositories {
        google()
        mavenCentral()
    }
}

val newBuildDir: Directory = rootProject.layout.buildDirectory.dir("../../build").get()
rootProject.layout.buildDirectory.value(newBuildDir)

subprojects {
    val newSubprojectBuildDir: Directory = newBuildDir.dir(project.name)
    project.layout.buildDirectory.value(newSubprojectBuildDir)
}
subprojects {
    project.evaluationDependsOn(":app")
}

tasks.register<Delete>("clean") {
    delete(rootProject.layout.buildDirectory)
}
