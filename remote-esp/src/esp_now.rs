use core::fmt::Debug;

use embassy_futures::join::join3;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_time::{Duration, Ticker};
use esp_hal::peripherals::WIFI;
use esp_radio::Controller;
use esp_radio::esp_now::{
    BROADCAST_ADDRESS, EspNowManager, EspNowReceiver, EspNowSender, EspNowWifiInterface, PeerInfo,
};
use esp_radio::wifi::WifiMode;
use log::{error, info};

use crate::make_static;

pub async fn communicate<
    MsgOutgoing: bincode::Encode + Debug,
    const OUTGOING_CHANNEL_SIZE: usize,
    MsgIncoming: bincode::Decode<()> + Debug,
    const INCOMING_CHANNEL_SIZE: usize,
>(
    wifi: WIFI<'static>,
    outgoing: Receiver<'static, NoopRawMutex, MsgOutgoing, OUTGOING_CHANNEL_SIZE>,
    incoming: Sender<'static, NoopRawMutex, MsgIncoming, INCOMING_CHANNEL_SIZE>,
) {
    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    let radio_controller = make_static!(Controller, radio_init);

    let (mut wifi_controller, interfaces) =
        esp_radio::wifi::new(radio_controller, wifi, Default::default())
            .expect("Failed to initialize Wi-Fi controller");
    wifi_controller.set_mode(WifiMode::Sta).unwrap();
    wifi_controller.start().unwrap();

    let esp_now = interfaces.esp_now;
    esp_now.set_channel(11).unwrap();

    info!("esp-now version {}", esp_now.version().unwrap());

    let (manager, esp_now_sender, esp_now_receiver) = esp_now.split();

    let broadcast_fut = broadcast(esp_now_sender, outgoing);
    let receive_fut = receive(&manager, esp_now_receiver, incoming);
    let fetch_peers_fut = fetch_peers(&manager);

    join3(broadcast_fut, receive_fut, fetch_peers_fut).await;
}

async fn broadcast<Msg: bincode::Encode + Debug, const CHANNEL_SIZE: usize>(
    mut sender: EspNowSender<'static>,
    messages: Receiver<'static, NoopRawMutex, Msg, CHANNEL_SIZE>,
) {
    loop {
        let message = messages.receive().await;
        let bytes = bincode::encode_to_vec(&message, bincode::config::standard()).unwrap();

        let status = sender.send_async(&BROADCAST_ADDRESS, &bytes).await;
        match status {
            Ok(_) => info!("Sent {message:?}"),
            Err(err) => error!("Error while sending: {err}"),
        }
    }
}

async fn receive<Msg: bincode::Decode<()> + Debug, const CHANNEL_SIZE: usize>(
    manager: &EspNowManager<'_>,
    mut receiver: EspNowReceiver<'static>,
    messages: Sender<'static, NoopRawMutex, Msg, CHANNEL_SIZE>,
) {
    loop {
        let received = receiver.receive_async().await;
        let (incoming_event, _) =
            bincode::decode_from_slice::<Msg, _>(received.data(), bincode::config::standard())
                .unwrap();
        info!("Received {:?}", incoming_event);

        messages.send(incoming_event).await;

        if received.info.dst_address == BROADCAST_ADDRESS {
            if !manager.peer_exists(&received.info.src_address) {
                manager
                    .add_peer(PeerInfo {
                        interface: EspNowWifiInterface::Sta,
                        peer_address: received.info.src_address,
                        lmk: None,
                        channel: None,
                        encrypt: false,
                    })
                    .unwrap();
                info!("Added peer {:?}", received.info.src_address);
            }
        }
    }
}

async fn fetch_peers(manager: &EspNowManager<'_>) {
    let mut ticker = Ticker::every(Duration::from_millis(500));
    loop {
        ticker.next().await;

        if manager.fetch_peer(false).is_err() {
            _ = manager.fetch_peer(true);
        }
    }
}
