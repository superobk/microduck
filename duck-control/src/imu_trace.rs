//! Optional bounded trace from the existing bus owner; disk IO is never on the tick.
use crate::imu::{DecodedSample, IMU_BLOCK_LEN};
use std::io::Write;
use std::sync::{
    Arc,
    atomic::{AtomicU64, Ordering},
    mpsc::{SyncSender, sync_channel},
};

#[derive(serde::Serialize)]
pub(crate) struct Record {
    pub schema_version: u32,
    pub host_read_start_ns: u64,
    pub host_read_end_ns: u64,
    /// Transport has accepted this ID/address payload; full wire packet is not captured.
    pub device_id: u8,
    pub register_address: u8,
    pub raw: [u8; IMU_BLOCK_LEN],
    pub decoded: DecodedSample,
    pub dropped_records: u64,
}

pub(crate) struct Trace {
    sender: SyncSender<Record>,
    dropped: Arc<AtomicU64>,
}

impl Trace {
    pub fn open(path: &std::path::Path) -> std::io::Result<Self> {
        let mut options = std::fs::OpenOptions::new();
        options.write(true).create_new(true);
        #[cfg(unix)]
        {
            use std::os::unix::fs::OpenOptionsExt;
            options.mode(0o600);
        }
        // Never append unrelated process clocks to an old capture by accident.
        let mut file = options.open(path)?;
        let (sender, receiver) = sync_channel::<Record>(256);
        let dropped = Arc::new(AtomicU64::new(0));
        std::thread::Builder::new()
            .name("imu-trace".into())
            .spawn(move || {
                let mut bytes = 0;
                for record in receiver {
                    let result = (|| -> std::io::Result<()> {
                        let mut line = serde_json::to_vec(&record)?;
                        line.push(b'\n');
                        if bytes + line.len() > 64 * 1024 * 1024 {
                            return Err(std::io::Error::other("64 MiB IMU trace limit reached"));
                        }
                        file.write_all(&line)?;
                        bytes += line.len();
                        Ok(())
                    })();
                    if let Err(error) = result {
                        tracing::warn!(%error,"IMU trace stopped; control unchanged");
                        break;
                    }
                }
            })?;
        Ok(Self { sender, dropped })
    }

    pub fn record(&self, mut record: Record) {
        record.dropped_records = self.dropped.load(Ordering::Relaxed);
        if self.sender.try_send(record).is_err() {
            self.dropped.fetch_add(1, Ordering::Relaxed);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn existing_capture_is_never_overwritten() {
        let p = std::env::temp_dir().join(format!(
            "duck-imu-trace-{}-{}.jsonl",
            std::process::id(),
            duck_ipc_proto::clock::monotonic_ns()
        ));
        std::fs::write(&p, b"keep this capture").unwrap();
        assert!(Trace::open(&p).is_err());
        assert_eq!(std::fs::read(&p).unwrap(), b"keep this capture");
        std::fs::remove_file(p).unwrap();
    }
}
