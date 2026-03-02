//! Flower swarm demo (compressed trajectories).
//!
//! Each Crazyflie in the swarm receives two compressed trajectories from JSON files:
//!   - `flower_trajectories/stem1.json`   → trajectory ID 1
//!   - `flower_trajectories/petals1.json` → trajectory ID 2
//!
//! The whole swarm runs in sync: arm → takeoff → stem → petals → land.
//!

use crazyflie_lib::subsystems::high_level_commander::TRAJECTORY_TYPE_POLY4D_COMPRESSED;
use crazyflie_lib::subsystems::memory::{CompressedSegment, CompressedStart, MemoryType, TrajectoryMemory};
use crazyflie_lib::{Crazyflie, TocCache};
use crazyflie_link::LinkContext;
use futures::future::join_all;
use std::fs;
use std::path::PathBuf;
use tokio::time::{sleep, Duration};

const URIS: &[&str] = &[
    "radio://0/80/2M/E7E7E7E7E7",
    "radio://0/80/2M/E7E7E7E7E8",
    "radio://0/80/2M/E7E7E7E7E9",
    "radio://0/80/2M/E7E7E7E7EA",
    "radio://0/80/2M/E7E7E7E7EB",
];


const TAKEOFF_HEIGHT: f32 = 0.3;
const TAKEOFF_DURATION: f32 = 2.0;
const TIME_SCALE: f32 = 1.0;
const STEM_ID: u8 = 1;
const PETALS_ID: u8 = 2;
const MAX_MEMORY: usize = 4096;

// Each drone gets its own petal color (hex)
const PETAL_COLORS: &[&str] = &[
    "#0e6cf1",
    "#ce8d00",
    "#f10e0e",
    "#f1e60e",
    "#ee0ef1",
    // Add more colors if you have more drones
];


const STEM_DURATION: f32 = 1.0;
const PETALS_DURATION: f32 = 10.0;

#[derive(Clone)]
struct FileTocCache {
    dir: PathBuf,
}

impl FileTocCache {
    fn new(dir: impl Into<PathBuf>) -> Self {
        let dir = dir.into();
        std::fs::create_dir_all(&dir).ok();
        Self { dir }
    }
}

impl TocCache for FileTocCache {
    fn get_toc(&self, crc32: u32) -> Option<String> {
        std::fs::read_to_string(self.dir.join(format!("{crc32:08x}.json"))).ok()
    }
    fn store_toc(&self, crc32: u32, toc: &str) {
        let _ = std::fs::write(self.dir.join(format!("{crc32:08x}.json")), toc);
    }
}

fn hex_color_to_wrgb8888(hex: &str) -> u32 {
    let hex = hex.trim_start_matches('#');
    let (r, g, b) = match hex.len() {
        6 => {
            let r = u8::from_str_radix(&hex[0..2], 16).unwrap_or(0);
            let g = u8::from_str_radix(&hex[2..4], 16).unwrap_or(0);
            let b = u8::from_str_radix(&hex[4..6], 16).unwrap_or(0);
            (r, g, b)
        }
        _ => (0, 0, 0),
    };
    (0u32 << 24) | ((r as u32) << 16) | ((g as u32) << 8) | (b as u32)
}


fn build_stem_compressed_for(idx: usize) -> Result<(CompressedStart, Vec<CompressedSegment>), Box<dyn std::error::Error>> {
    let path = format!("flower_trajectories/stem{}.json", idx + 1);
    let data = fs::read_to_string(&path)?;
    let points: Vec<[f32; 3]> = serde_json::from_str(&data)?;
    let dt = STEM_DURATION / (points.len() - 1) as f32;
    let [sx0, sy0, sz0] = points[0];
    let start = CompressedStart::new(0.0, 0.0, 0.0, 0.0);
    let mut segments = Vec::new();
    for window in points.windows(2) {
        let [x1, y1, z1] = window[1];
        segments.push(CompressedSegment::new(dt, vec![x1 - sx0], vec![y1 - sy0], vec![z1 - sz0], vec![])?);
    }
    Ok((start, segments))
}

fn build_petals_compressed_for(idx: usize) -> Result<(CompressedStart, Vec<CompressedSegment>), Box<dyn std::error::Error>> {
    let path = format!("flower_trajectories/petals{}.json", idx + 1);
    let data = fs::read_to_string(&path)?;
    let groups: Vec<Vec<[f32; 3]>> = serde_json::from_str(&data)?;
    let segment_count = groups.iter().map(|g| g.len().saturating_sub(1)).sum::<usize>() + (groups.len() - 1);
    let dt = PETALS_DURATION / segment_count as f32;
    let [px0, py0, pz0] = groups[0][0];
    let start = CompressedStart::new(0.0, 0.0, 0.0, 0.0);
    let mut segments = Vec::new();
    for window in groups[0].windows(2) {
        let [x1, y1, z1] = window[1];
        segments.push(CompressedSegment::new(dt, vec![x1 - px0], vec![y1 - py0], vec![z1 - pz0], vec![])?);
    }
    for group in &groups[1..] {
        let [x0, y0, z0] = group[0];
        segments.push(CompressedSegment::new(dt, vec![x0 - px0], vec![y0 - py0], vec![z0 - pz0], vec![])?);
        for window in group.windows(2) {
            let [x1, y1, z1] = window[1];
            segments.push(CompressedSegment::new(dt, vec![x1 - px0], vec![y1 - py0], vec![z1 - pz0], vec![])?);
        }
    }
    Ok((start, segments))
}

async fn upload_trajectories(
    cf: &Crazyflie,
    stem: &(CompressedStart, Vec<CompressedSegment>),
    petals: &(CompressedStart, Vec<CompressedSegment>),
    petals_offset: usize,
) -> crazyflie_lib::Result<()> {
    let dev = cf.memory.get_memories(Some(MemoryType::Trajectory))[0].clone();
    let mem = cf.memory.open_memory::<TrajectoryMemory>(dev).await.unwrap()?;
    mem.write_compressed(&stem.0, &stem.1, 0).await?;
    mem.write_compressed(&petals.0, &petals.1, petals_offset).await?;
    cf.memory.close_memory(mem).await?;
    Ok(())
}

async fn swarm_flight_sequence(
    cfs: &[Crazyflie],
    stems: &[ (CompressedStart, Vec<CompressedSegment>) ],
    petals: &[ (CompressedStart, Vec<CompressedSegment>) ],
) -> Result<(), Box<dyn std::error::Error>> {
    // Each drone gets its own stem/petals
    let petals_offsets: Vec<_> = stems.iter().map(|stem| 8 + stem.1.len() * 9).collect();
    let stem_lens: Vec<_> = stems.iter().map(|stem| (1 + stem.1.len()) as u8).collect();
    let petals_lens: Vec<_> = petals.iter().map(|petal| (1 + petal.1.len()) as u8).collect();

    println!("\nUploading compressed trajectories...");
    join_all(cfs.iter().enumerate().map(|(i, cf)| upload_trajectories(cf, &stems[i], &petals[i], petals_offsets[i])))
        .await
        .into_iter()
        .collect::<crazyflie_lib::Result<Vec<_>>>()?;
    for i in 0..cfs.len() {
        println!(
            "Drone {}: {} bytes ({} stem + {} petals)",
            i+1,
            petals_offsets[i] + 8 + petals[i].1.len() * 9,
            8 + stems[i].1.len() * 9,
            8 + petals[i].1.len() * 9,
        );
    }

    join_all(cfs.iter().enumerate().map(|(i, cf)| {
        let stem_lens = stem_lens.clone();
        let petals_offsets = petals_offsets.clone();
        let petals_lens = petals_lens.clone();
        async move {
            cf.high_level_commander
                .define_trajectory(STEM_ID, 0, stem_lens[i], Some(TRAJECTORY_TYPE_POLY4D_COMPRESSED))
                .await?;
            cf.high_level_commander
                .define_trajectory(PETALS_ID, petals_offsets[i] as u32, petals_lens[i], Some(TRAJECTORY_TYPE_POLY4D_COMPRESSED))
                .await
        }
    }))
    .await
    .into_iter()
    .collect::<crazyflie_lib::Result<Vec<_>>>()?;
    println!("Trajectories defined");
    sleep(Duration::from_secs(1)).await;

    join_all(cfs.iter().map(|cf| async move {
        let _ = cf.param.set("led.bitmask", 128u8).await;
    })).await;

    println!("\nArming...");
    join_all(cfs.iter().map(|cf| async move {
        let _ = cf.platform.send_arming_request(true).await;
    }))
    .await;
    sleep(Duration::from_millis(300)).await;
    println!("Armed!");

    println!("\nTaking off...");
    let green = hex_color_to_wrgb8888("#13ee0b");
    join_all(cfs.iter().map(|cf| async move {
        let _ = cf.param.set("colorLedBot.wrgb8888", green).await;
        let _ = cf.param.set("colorLedTop.wrgb8888", green).await;
        let _ = cf.high_level_commander
            .take_off(TAKEOFF_HEIGHT, None, TAKEOFF_DURATION, None)
            .await;
    }))
    .await;
    sleep(Duration::from_secs_f32(TAKEOFF_DURATION)).await;

    println!("Starting stem trajectory...");
    join_all(cfs.iter().map(|cf| async move {
        let _ = cf.param.set("colorLedBot.wrgb8888", green).await;
        let _ = cf.param.set("colorLedTop.wrgb8888", green).await;
        let _ = cf.high_level_commander
            .start_trajectory(STEM_ID, TIME_SCALE, true, false, false, None)
            .await;
    }))
    .await;
    sleep(Duration::from_secs(STEM_DURATION.ceil() as u64 + 1)).await;

    println!("Starting petals trajectory...");
    join_all(cfs.iter().enumerate().map(|(i, cf)| {
        let color_hex = PETAL_COLORS.get(i).unwrap_or(&"#f10e0e");
        let color = hex_color_to_wrgb8888(color_hex);
        async move {
            let _ = cf.param.set("colorLedBot.wrgb8888", color).await;
            let _ = cf.param.set("colorLedTop.wrgb8888", color).await;
            let _ = cf.high_level_commander
                .start_trajectory(PETALS_ID, TIME_SCALE, true, false, false, None)
                .await;
        }
    }))
    .await;
    sleep(Duration::from_secs(PETALS_DURATION.ceil() as u64 + 1)).await;

    println!("Landing...");
    let off = hex_color_to_wrgb8888("#000000");
    join_all(cfs.iter().map(|cf| async move {
        let _ = cf.param.set("colorLedBot.wrgb8888", off).await;
        let _ = cf.param.set("colorLedTop.wrgb8888", off).await;
        let _ = cf.high_level_commander.land(0.0, None, TAKEOFF_DURATION, None).await;
    }))
    .await;
    sleep(Duration::from_secs_f32(TAKEOFF_DURATION)).await;

    join_all(cfs.iter().map(|cf| async move {
        let _ = cf.param.set("led.bitmask", 0u8).await;
        let _ = cf.high_level_commander.stop(None).await;
    }))
    .await;

    Ok(())
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Build per-drone trajectories
    let stems: Vec<_> = (0..URIS.len())
        .map(|i| build_stem_compressed_for(i))
        .collect::<Result<_, _>>()?;
    let petals: Vec<_> = (0..URIS.len())
        .map(|i| build_petals_compressed_for(i))
        .collect::<Result<_, _>>()?;

    // Print info for each drone
    for i in 0..URIS.len() {
        let stem_bytes = 8 + stems[i].1.len() * 9;
        let petals_bytes = 8 + petals[i].1.len() * 9;
        let total_bytes = stem_bytes + petals_bytes;
        if total_bytes > MAX_MEMORY {
            return Err(format!(
                "Drone {}: Trajectories too large: {total_bytes} bytes > {MAX_MEMORY} max",
                i+1
            ).into());
        }
        println!("Drone {}: stem: {} segments ({} bytes)", i+1, stems[i].1.len(), stem_bytes);
        println!("Drone {}: petals: {} segments ({} bytes)", i+1, petals[i].1.len(), petals_bytes);
    }

    println!("\nConnecting to {} drones...", URIS.len());
    let ctx = LinkContext::new();
    let cache = FileTocCache::new("cache");
    let cfs: Vec<Crazyflie> = join_all(
        URIS.iter()
            .map(|uri| Crazyflie::connect_from_uri(&ctx, uri, cache.clone())),
    )
    .await
    .into_iter()
    .collect::<crazyflie_lib::Result<_>>()?;
    println!("All connected!");

    let result = swarm_flight_sequence(&cfs, &stems, &petals).await;
    if let Err(ref e) = result {
        eprintln!("\nFlight error: {e}");
    }
    println!("\nDisarming...");
    join_all(cfs.iter().map(|cf| cf.platform.send_arming_request(false))).await;
    println!("Disconnecting...");
    join_all(cfs.iter().map(|cf| cf.disconnect())).await;
    println!("Done!");
    result
}
