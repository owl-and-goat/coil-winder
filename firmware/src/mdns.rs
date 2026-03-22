//! mDNS responder for `.local` hostname resolution.

use core::net::{Ipv4Addr, Ipv6Addr};

use defmt::*;
use edge_mdns::buf::VecBufAccess;
use edge_mdns::domain::base::Ttl;
use edge_mdns::host::Host;
use edge_mdns::io::{self, IPV4_DEFAULT_SOCKET};
use edge_mdns::HostAnswersMdnsHandler;
use edge_nal::UdpSplit;
use edge_nal_embassy::{Udp, UdpBuffers};
use embassy_net::Stack;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use rand::RngCore;

pub async fn run<R: RngCore>(
    stack: Stack<'static>,
    hostname: &'static str,
    ip: Ipv4Addr,
    mut rng: R,
) -> ! {
    info!("mDNS: starting responder for {}.local -> {}", hostname, ip);

    let buffers = UdpBuffers::<1, 1024, 1024, 2>::new();
    let udp = Udp::new(stack, &buffers);

    let (recv_buf, send_buf) = (
        VecBufAccess::<CriticalSectionRawMutex, 1024>::new(),
        VecBufAccess::<CriticalSectionRawMutex, 1024>::new(),
    );

    loop {
        match run_mdns(&udp, &recv_buf, &send_buf, hostname, ip, &mut rng).await {
            Ok(()) => {
                warn!("mDNS: responder exited unexpectedly, restarting");
            }
            Err(e) => {
                error!("mDNS: error: {:?}, restarting", Debug2Format(&e));
                embassy_time::Timer::after_secs(1).await;
            }
        }
    }
}

async fn run_mdns<T, RB, SB, R>(
    stack: &T,
    recv_buf: &RB,
    send_buf: &SB,
    hostname: &str,
    ip: Ipv4Addr,
    rng: &mut R,
) -> Result<(), io::MdnsIoError<T::Error>>
where
    T: edge_nal::UdpBind,
    RB: edge_mdns::buf::BufferAccess<[u8]>,
    SB: edge_mdns::buf::BufferAccess<[u8]>,
    R: RngCore,
{
    debug!("mDNS: binding to {:?}", IPV4_DEFAULT_SOCKET);
    let mut socket = io::bind(
        stack,
        IPV4_DEFAULT_SOCKET,
        Some(Ipv4Addr::UNSPECIFIED),
        None,
    )
    .await?;
    debug!("mDNS: bound successfully, joining multicast");

    let (recv, send) = socket.split();
    debug!("mDNS: socket split, starting responder loop");

    let host = Host {
        hostname,
        ipv4: ip,
        ipv6: Ipv6Addr::UNSPECIFIED,
        ttl: Ttl::from_secs(60),
    };

    let signal = Signal::<CriticalSectionRawMutex, ()>::new();

    let mdns = io::Mdns::new(
        Some(Ipv4Addr::UNSPECIFIED),
        None, // No IPv6
        recv,
        send,
        recv_buf,
        send_buf,
        rng,
        &signal,
    );

    info!("mDNS: advertising {}.local", hostname);
    mdns.run(HostAnswersMdnsHandler::new(&host)).await
}
