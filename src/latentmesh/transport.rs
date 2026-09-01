//! Small async datagram transports for encoded LatentMesh SparseRadioFrames.

use std::{io, net::SocketAddr};

use async_trait::async_trait;
use latentmesh_air_core::FRAME_MAX_BYTES;
use tokio::{net::UdpSocket, sync::mpsc};

#[derive(Debug, thiserror::Error)]
pub enum TransportError {
    #[error("frame transport is closed")]
    Closed,
    #[error("frame length {length} exceeds transport bound {maximum}")]
    FrameTooLarge { length: usize, maximum: usize },
    #[error("channel capacity must be between 1 and 4096")]
    InvalidCapacity,
    #[error("UDP transport error: {0}")]
    Io(#[from] io::Error),
}

pub type TransportResult<T> = Result<T, TransportError>;

/// One transport record equals one encoded SparseRadioFrame. Implementations
/// must preserve datagram boundaries and enforce the Air frame size bound.
#[async_trait]
pub trait FrameTransport: Send {
    async fn send_frame(&self, frame: &[u8]) -> TransportResult<()>;
    async fn receive_frame(&mut self) -> TransportResult<Vec<u8>>;
}

/// A bounded, bidirectional in-process link for simulation and deterministic
/// integration tests. Backpressure is provided by Tokio's bounded channel.
#[derive(Debug)]
pub struct ChannelFrameTransport {
    sender: mpsc::Sender<Vec<u8>>,
    receiver: mpsc::Receiver<Vec<u8>>,
}

/// Construct two cross-connected transport endpoints.
pub fn bounded_channel_loopback(
    capacity: usize,
) -> TransportResult<(ChannelFrameTransport, ChannelFrameTransport)> {
    if !(1..=4_096).contains(&capacity) {
        return Err(TransportError::InvalidCapacity);
    }
    let (a_to_b_tx, a_to_b_rx) = mpsc::channel(capacity);
    let (b_to_a_tx, b_to_a_rx) = mpsc::channel(capacity);
    Ok((
        ChannelFrameTransport {
            sender: a_to_b_tx,
            receiver: b_to_a_rx,
        },
        ChannelFrameTransport {
            sender: b_to_a_tx,
            receiver: a_to_b_rx,
        },
    ))
}

#[async_trait]
impl FrameTransport for ChannelFrameTransport {
    async fn send_frame(&self, frame: &[u8]) -> TransportResult<()> {
        validate_frame_bound(frame)?;
        self.sender
            .send(frame.to_vec())
            .await
            .map_err(|_| TransportError::Closed)
    }

    async fn receive_frame(&mut self) -> TransportResult<Vec<u8>> {
        let frame = self.receiver.recv().await.ok_or(TransportError::Closed)?;
        validate_frame_bound(&frame)?;
        Ok(frame)
    }
}

/// Connected UDP datagram transport. Connecting the socket pins the peer and
/// lets the operating system discard datagrams from other source addresses.
#[derive(Debug)]
pub struct UdpFrameTransport {
    socket: UdpSocket,
}

impl UdpFrameTransport {
    pub async fn bind(local: SocketAddr, peer: SocketAddr) -> TransportResult<Self> {
        let socket = UdpSocket::bind(local).await?;
        socket.connect(peer).await?;
        Ok(Self { socket })
    }

    pub async fn from_socket(socket: UdpSocket, peer: SocketAddr) -> TransportResult<Self> {
        socket.connect(peer).await?;
        Ok(Self { socket })
    }

    pub fn local_addr(&self) -> TransportResult<SocketAddr> {
        Ok(self.socket.local_addr()?)
    }
}

#[async_trait]
impl FrameTransport for UdpFrameTransport {
    async fn send_frame(&self, frame: &[u8]) -> TransportResult<()> {
        validate_frame_bound(frame)?;
        let sent = self.socket.send(frame).await?;
        if sent != frame.len() {
            return Err(TransportError::Io(io::Error::new(
                io::ErrorKind::WriteZero,
                "partial UDP datagram send",
            )));
        }
        Ok(())
    }

    async fn receive_frame(&mut self) -> TransportResult<Vec<u8>> {
        // One extra byte distinguishes an exact maximum frame from an
        // oversized datagram truncated by the operating system.
        let mut buffer = [0_u8; FRAME_MAX_BYTES + 1];
        let received = self.socket.recv(&mut buffer).await?;
        if received > FRAME_MAX_BYTES {
            return Err(TransportError::FrameTooLarge {
                length: received,
                maximum: FRAME_MAX_BYTES,
            });
        }
        Ok(buffer[..received].to_vec())
    }
}

fn validate_frame_bound(frame: &[u8]) -> TransportResult<()> {
    if frame.len() > FRAME_MAX_BYTES {
        return Err(TransportError::FrameTooLarge {
            length: frame.len(),
            maximum: FRAME_MAX_BYTES,
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn bounded_channel_is_bidirectional_and_preserves_datagrams() {
        let (mut left, mut right) = bounded_channel_loopback(2).unwrap();
        left.send_frame(&[1, 2, 3]).await.unwrap();
        assert_eq!(right.receive_frame().await.unwrap(), vec![1, 2, 3]);
        right.send_frame(&[4, 5]).await.unwrap();
        assert_eq!(left.receive_frame().await.unwrap(), vec![4, 5]);
    }

    #[tokio::test]
    async fn channel_rejects_oversized_input_before_queueing() {
        let (left, _right) = bounded_channel_loopback(1).unwrap();
        let oversized = vec![0_u8; FRAME_MAX_BYTES + 1];
        assert!(matches!(
            left.send_frame(&oversized).await,
            Err(TransportError::FrameTooLarge {
                length,
                maximum: FRAME_MAX_BYTES
            }) if length == FRAME_MAX_BYTES + 1
        ));
    }

    #[tokio::test]
    async fn connected_udp_loopback_preserves_datagram_boundaries() {
        let left_socket = UdpSocket::bind("127.0.0.1:0").await.unwrap();
        let right_socket = UdpSocket::bind("127.0.0.1:0").await.unwrap();
        let left_addr = left_socket.local_addr().unwrap();
        let right_addr = right_socket.local_addr().unwrap();
        let mut left = UdpFrameTransport::from_socket(left_socket, right_addr)
            .await
            .unwrap();
        let mut right = UdpFrameTransport::from_socket(right_socket, left_addr)
            .await
            .unwrap();

        left.send_frame(&[1, 2, 3, 4]).await.unwrap();
        assert_eq!(right.receive_frame().await.unwrap(), vec![1, 2, 3, 4]);
        right.send_frame(&[5, 6]).await.unwrap();
        assert_eq!(left.receive_frame().await.unwrap(), vec![5, 6]);
    }
}
