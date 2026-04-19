use serde::{Deserialize, Serialize};
use snow::{Builder, TransportState};
use std::collections::HashMap;

const NOISE_PATTERN: &str = "Noise_XX_25519_AESGCM_SHA256";

pub const HANDSHAKE_TAG: u8 = 0x01;
pub const TRANSPORT_TAG: u8 = 0x02;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum MsgType {
    Hello,
    Heartbeat,
    MapDiff,
}

/// Wire-level application message. Heartbeats are sent plaintext.
/// Map diffs are serialized into this struct, then encrypted via `NoiseSessionTable`.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Message {
    pub msg_type: MsgType,
    pub sender_id: u32,
    pub sender_pos: (f32, f32),
    pub path: Option<Vec<(f32, f32)>>,
    pub peer_list: Vec<u32>,
    pub payload: Vec<u8>,
    pub timestamp: u64,
}

/// Known live peer tracked in a drone's routing table.
#[derive(Debug, Clone)]
pub struct Peer {
    pub id: u32,
    pub last_seen: u64,
    pub position: (f32, f32),
    pub path: Option<Vec<(f32, f32)>>,
}

impl Message {
    pub fn to_bytes(&self) -> Result<Vec<u8>, bincode::Error> {
        bincode::serialize(self)
    }

    pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
        bincode::deserialize(bytes).ok()
    }
}

/// Generates a fresh Noise static keypair for a drone instance.
/// Each drone gets a unique identity keypair on startup.
///
/// # Errors
/// Returns an error if the OS cannot provide cryptographic entropy or if the
/// Noise pattern string is invalid.
pub fn generate_keypair() -> anyhow::Result<(Vec<u8>, Vec<u8>)> {
    let params: snow::params::NoiseParams = NOISE_PATTERN.parse()
        .map_err(|e| anyhow::anyhow!("Invalid Noise pattern: {}", e))?;
    let builder = Builder::new(params);
    let keypair = builder.generate_keypair()
        .map_err(|e| anyhow::anyhow!("Keypair generation failed (entropy unavailable?): {}", e))?;
    Ok((keypair.private, keypair.public))
}

/// A completed Noise transport session with a specific peer.
/// The `transport` field is intentionally private — all crypto must
/// flow through `NoiseSessionTable::encrypt` / `decrypt`.
pub struct NoiseSession {
    transport: TransportState,
}

/// Tracks all per-peer Noise XX sessions for a single drone.
/// Handles the 3-message handshake and transitions to encrypted transport.
///
/// # Role assignment
/// To prevent both sides racing to be initiator simultaneously, the drone with
/// the *lower* ID always acts as responder; the *higher* ID initiates.
pub struct NoiseSessionTable {
    local_private_key: Vec<u8>,
    /// Completed encrypted sessions keyed by peer ID.
    pub sessions: HashMap<u32, NoiseSession>,
    /// In-progress XX handshake states keyed by peer ID.
    pub(crate) handshakes: HashMap<u32, snow::HandshakeState>,
}

impl NoiseSessionTable {
    pub fn new(local_private_key: Vec<u8>) -> Self {
        Self {
            local_private_key,
            sessions: HashMap::new(),
            handshakes: HashMap::new(),
        }
    }

    /// Returns true if we have a completed encrypted session with this peer.
    pub fn has_session(&self, peer_id: u32) -> bool {
        self.sessions.contains_key(&peer_id)
    }

    /// Initiates an outbound Noise XX handshake.
    /// Returns the first handshake message bytes to send to the peer.
    ///
    /// # Errors
    /// Returns an error if the Noise pattern string is invalid or the key is malformed.
    pub fn initiate_handshake(&mut self, peer_id: u32) -> anyhow::Result<Vec<u8>> {
        let params: snow::params::NoiseParams = NOISE_PATTERN.parse()?;
        let mut hs = Builder::new(params)
            .local_private_key(&self.local_private_key)
            .build_initiator()?;

        let mut msg_buf = vec![0u8; 1024];
        let len = hs.write_message(&[], &mut msg_buf)?;
        msg_buf.truncate(len);

        self.handshakes.insert(peer_id, hs);
        Ok(msg_buf)
    }

    /// Processes an inbound handshake message from a peer.
    /// Returns `Some(reply_bytes)` if a response needs to be sent back.
    /// Returns `None` when the handshake is complete and the session is promoted.
    ///
    /// # Errors
    /// Returns an error if the message is malformed or fails cryptographic verification.
    pub fn process_handshake_msg(&mut self, peer_id: u32, incoming: &[u8]) -> anyhow::Result<Option<Vec<u8>>> {
        // Handshake messages in Noise XX are always under 100 bytes.
        // Use stack buffers to avoid 64KB heap allocations on every message.
        let mut msg_buf = [0u8; 1024];
        let mut payload_buf = [0u8; 256];

        if let Some(hs) = self.handshakes.get_mut(&peer_id) {
            // Continue existing handshake as initiator (receiving step 2 from responder)
            hs.read_message(incoming, &mut payload_buf)?;

            if hs.is_handshake_finished() {
                let hs = self.handshakes.remove(&peer_id)
                    .expect("handshake state must exist: guarded by get_mut above");
                let transport = hs.into_transport_mode()?;
                self.sessions.insert(peer_id, NoiseSession { transport });
                println!("Noise handshake complete with peer {}. Session encrypted.", peer_id);
                return Ok(None);
            }

            // Initiator sends step 3
            let len = hs.write_message(&[], &mut msg_buf)?;
            return Ok(Some(msg_buf[..len].to_vec()));
        }

        // We are the RESPONDER — first message from this peer
        if !self.sessions.contains_key(&peer_id) {
            let params: snow::params::NoiseParams = NOISE_PATTERN.parse()?;
            let mut hs = Builder::new(params)
                .local_private_key(&self.local_private_key)
                .build_responder()?;

            hs.read_message(incoming, &mut payload_buf)?;

            let len = hs.write_message(&[], &mut msg_buf)?;

            if hs.is_handshake_finished() {
                let transport = hs.into_transport_mode()?;
                self.sessions.insert(peer_id, NoiseSession { transport });
                println!("Noise handshake complete with peer {} (responder). Session encrypted.", peer_id);
                return Ok(Some(msg_buf[..len].to_vec()));
            }

            self.handshakes.insert(peer_id, hs);
            return Ok(Some(msg_buf[..len].to_vec()));
        }

        Ok(None)
    }

    /// Encrypts a plaintext payload for a peer using the established Noise transport.
    ///
    /// # Errors
    /// Returns an error if no session exists for the peer or encryption fails.
    pub fn encrypt(&mut self, peer_id: u32, plaintext: &[u8]) -> anyhow::Result<Vec<u8>> {
        let session = self.sessions.get_mut(&peer_id)
            .ok_or_else(|| anyhow::anyhow!("No session for peer {}", peer_id))?;
        let mut buf = vec![0u8; plaintext.len() + 1024];
        let len = session.transport.write_message(plaintext, &mut buf)?;
        buf.truncate(len);
        Ok(buf)
    }

    /// Decrypts a ciphertext payload from a peer using the established Noise transport.
    ///
    /// # Errors
    /// Returns an error if no session exists for the peer or decryption/authentication fails.
    pub fn decrypt(&mut self, peer_id: u32, ciphertext: &[u8]) -> anyhow::Result<Vec<u8>> {
        let session = self.sessions.get_mut(&peer_id)
            .ok_or_else(|| anyhow::anyhow!("No session for peer {}", peer_id))?;
        let mut buf = vec![0u8; ciphertext.len() + 1024];
        let len = session.transport.read_message(ciphertext, &mut buf)?;
        buf.truncate(len);
        Ok(buf)
    }
}

/// UDP wire envelope wrapping either a Noise handshake fragment or an encrypted transport payload.
#[derive(Serialize, Deserialize)]
pub struct Envelope {
    pub tag: u8,       // HANDSHAKE_TAG or TRANSPORT_TAG
    pub sender_id: u32,
    pub payload: Vec<u8>,
}

impl Envelope {
    pub fn to_bytes(&self) -> Result<Vec<u8>, bincode::Error> {
        bincode::serialize(self)
    }

    pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
        bincode::deserialize(bytes).ok()
    }
}
