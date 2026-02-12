package quadstorm.remote

import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbManager
import android.os.Bundle
import android.util.Log
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.tooling.preview.PreviewScreenSizes
import androidx.compose.ui.unit.dp
import quadstorm.remote.ui.theme.QuadstormRemoteTheme

private const val ACTION_USB_PERMISSION = "quadstorm.remote.USB_PERMISSION"

private val usbReceiver = object : BroadcastReceiver() {
    override fun onReceive(context: Context, intent: Intent) {
        if (ACTION_USB_PERMISSION != intent.action) return

        synchronized(this) {
            val device = intent.getParcelableExtra(
                UsbManager.EXTRA_DEVICE,
                UsbDevice::class.java,
            )

            if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
                device?.apply {
                    Log.d("USB", deviceProtocol.toString())
                    // call method to set up device communication
                }
            } else {
                Log.d("USB", "permission denied for device $device")
            }
        }
    }
}

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val manager = getSystemService(Context.USB_SERVICE) as UsbManager
        val permissionIntent = PendingIntent.getBroadcast(
            this,
            0,
            Intent(ACTION_USB_PERMISSION),
            PendingIntent.FLAG_IMMUTABLE,
        )
        val filter = IntentFilter(ACTION_USB_PERMISSION)
        registerReceiver(usbReceiver, filter)

        for (usbDevice in manager.deviceList.values) {
            if (usbDevice.vendorId == 12346 && usbDevice.productId == 4097) {
                manager.requestPermission(usbDevice, permissionIntent)
            }
        }

        enableEdgeToEdge()
        setContent {
            QuadstormRemoteTheme {
                QuadstormRemoteApp()
            }
        }
    }
}

@PreviewScreenSizes
@Composable
fun QuadstormRemoteApp() {
    Scaffold(modifier = Modifier.fillMaxSize()) { innerPadding ->
        Box(modifier = Modifier.padding(innerPadding)) {
            HomeScreen()
        }
    }
}

@Composable
fun HomeScreen() {
    Row(
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically,
        modifier = Modifier.fillMaxSize()
    ) {
        Text(
            "Height", modifier = Modifier
                .fillMaxHeight()
                .weight(1f)
                .background(Color.Green)
        )
        Text(
            "Logging", modifier = Modifier
                .fillMaxHeight()
                .width(400.dp)
                .background(Color.Blue)
        )
        Text(
            "Direction", modifier = Modifier
                .fillMaxHeight()
                .weight(1f)
                .background(Color.Red)
        )
    }
}

@Preview
@Composable
fun Preview() {
    QuadstormRemoteTheme {
        HomeScreen()
    }
}