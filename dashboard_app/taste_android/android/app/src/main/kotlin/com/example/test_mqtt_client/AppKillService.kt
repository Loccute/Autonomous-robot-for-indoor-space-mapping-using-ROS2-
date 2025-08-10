package com.example.test_mqtt_client

import android.app.Service
import android.content.Intent
import android.os.Build
import android.os.IBinder
import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import com.google.firebase.database.FirebaseDatabase
import android.util.Log


class AppKillService : Service() {

    override fun onCreate() {
        super.onCreate()
        startForeground(1, createNotification())
        Log.d("AppKillService", "Service đang chạy foreground")
    }

    override fun onTaskRemoved(rootIntent: Intent?) {
        Log.d("AppKillService", "App bị vuốt kill, cập nhật Firebase")

        val ref = FirebaseDatabase.getInstance().getReference("data/users")
        ref.child("using").setValue(false)
        ref.child("lastActiveTime").setValue(System.currentTimeMillis())

        stopSelf()
        super.onTaskRemoved(rootIntent)
    }

    private fun createNotification(): Notification {
        val channelId = "kill_monitor"

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val channel = NotificationChannel(
                channelId,
                "App Kill Monitor",
                NotificationManager.IMPORTANCE_HIGH 
            )
            val manager = getSystemService(NotificationManager::class.java)
            manager?.createNotificationChannel(channel)
        }

        return Notification.Builder(this, channelId)
            .setContentTitle("App đang chạy nền")
            .setContentText("Giám sát trạng thái app...")
            .setSmallIcon(R.mipmap.ic_launcher) // đảm bảo có icon
            .build()
    }

    override fun onBind(intent: Intent?): IBinder? = null
}
