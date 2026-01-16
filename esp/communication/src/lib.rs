#![no_std]

extern crate alloc;

use alloc::string::String;
use core::fmt::Debug;

use embassy_futures::join::join3;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_time::{Duration, Ticker};
use esp_hal::peripherals::WIFI;
use esp_radio::esp_now::{
    BROADCAST_ADDRESS, EspNowManager, EspNowReceiver, EspNowSender, EspNowWifiInterface, PeerInfo,
};
use esp_radio::wifi::WifiMode;
use log::{error, info};
use wincode::{SchemaRead, SchemaReadOwned, SchemaWrite};

#[derive(Debug, SchemaWrite, SchemaRead)]
pub enum RemoteRequest {
    Ping,
    PowerOn,
    PowerOff,
    Move {
        /// left (-1) to right (+1)
        x: f32,
        /// backwards (-1) to forwards (+1)
        y: f32,
        /// down (-1) to up (+1)
        z: f32,
    },
}

#[derive(Debug, SchemaWrite, SchemaRead)]
pub enum QuadcopterResponse {
    Pong,
    Log(String),
}

pub async fn communicate<
    MsgOutgoing: SchemaWrite<Src = MsgOutgoing> + Debug,
    const OUTGOING_CHANNEL_SIZE: usize,
    MsgIncoming: SchemaReadOwned<Dst = MsgIncoming> + Debug,
    const INCOMING_CHANNEL_SIZE: usize,
>(
    wifi: WIFI<'_>,
    outgoing: Receiver<'_, NoopRawMutex, MsgOutgoing, OUTGOING_CHANNEL_SIZE>,
    incoming: Sender<'_, NoopRawMutex, MsgIncoming, INCOMING_CHANNEL_SIZE>,
) {
    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");

    let (mut wifi_controller, interfaces) =
        esp_radio::wifi::new(&radio_init, wifi, Default::default())
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

async fn broadcast<Msg: SchemaWrite<Src = Msg> + Debug, const CHANNEL_SIZE: usize>(
    mut sender: EspNowSender<'_>,
    messages: Receiver<'_, NoopRawMutex, Msg, CHANNEL_SIZE>,
) {
    loop {
        let message = messages.receive().await;
        let bytes = wincode::serialize(&message).unwrap();

        let status = sender.send_async(&BROADCAST_ADDRESS, &bytes).await;
        match status {
            Ok(_) => info!("Sent {message:?}"),
            Err(err) => error!("Error while sending: {err}"),
        }
    }
}

async fn receive<'a, Msg: SchemaReadOwned<Dst = Msg> + Debug, const CHANNEL_SIZE: usize>(
    manager: &EspNowManager<'_>,
    mut receiver: EspNowReceiver<'_>,
    messages: Sender<'_, NoopRawMutex, Msg, CHANNEL_SIZE>,
) {
    loop {
        let received = receiver.receive_async().await;
        let incoming_event = wincode::deserialize(received.data()).unwrap();
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
