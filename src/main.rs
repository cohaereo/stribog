use rerun::{
    Radius,
    components::GeoLineString,
    external::re_types::blueprint::{
        archetypes::{MapBackground, MapZoom},
        components::{MapProvider, ZoomLevel},
        views::MapView,
    },
};
use rtlsdr_rs::{RtlSdr, TunerGain};
use std::{
    borrow::Cow,
    collections::{HashMap, HashSet},
    error,
    io::{BufWriter, Seek, Write},
    ops::Range,
    time::Instant,
};

use crate::{
    aircraft::{Aircraft, Icao},
    bitreader::BitReader,
};
mod aircraft;
mod bitreader;
mod cpr;

const DEFAULT_BUF_LENGTH: usize = (16 * 16384);
// const DEFAULT_BUF_LENGTH: usize = 2_500_000;

const DEVICE_INDEX: usize = 0;
const SAMPLE_RATE: u32 = 2_000_000;

#[macro_use]
extern crate log;

fn main() -> anyhow::Result<()> {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .format_timestamp(None)
        .init();

    // Open device
    let mut sdr = RtlSdr::open(DEVICE_INDEX).expect("Unable to open SDR device!");
    // info!("{:#?}", sdr);

    let gains = sdr.get_tuner_gains()?;
    info!(
        "Supported gain values ({}): {:?}",
        gains.len(),
        gains,
        // gains
        //     .iter()
        //     .map(|g| { *g as f32 / 10.0 })
        //     .collect::<Vec<_>>()
    );

    // sdr.set_direct_sampling(rtlsdr_rs::DirectSampleMode::On)?;
    // Set sample rate
    sdr.set_sample_rate(SAMPLE_RATE)?;
    info!("Sampling at {} S/s", sdr.get_sample_rate());

    // sdr.set_tuner_bandwidth(3_000_000)?;
    sdr.set_bias_tee(false)?;
    // sdr.set_tuner_gain(TunerGain::Manual(328))?;
    sdr.set_tuner_gain(TunerGain::Manual(496))?;
    // sdr.set_tuner_gain(TunerGain::Auto)?;

    // sdr.set_center_freq(1_030_000_000)?; // Set center frequency to 1.090 GHz
    sdr.set_freq_correction(0)?;
    sdr.set_center_freq(1_090_000_000)?; // Set center frequency to 1.090 GHz

    // Reset the endpoint before we try to read from it (mandatory)
    info!("Reset buffer");
    sdr.reset_buffer()?;

    let mut bitmap =
        BufWriter::new(std::fs::File::create("bitmap.data").expect("Unable to create dump file"));

    // let mut raw =
    //     BufWriter::new(std::fs::File::create("dump.bin").expect("Unable to create dump file"));

    // let mut interrogators = HashSet::<u32>::default();
    info!("Reading samples in sync mode...");
    let mut buf = vec![0u8; DEFAULT_BUF_LENGTH];
    let mut aircrafts = HashMap::<Icao, Aircraft>::new();
    let rec = rerun::RecordingStreamBuilder::new("stribog").connect_grpc()?;
    // rec.log_static(
    //     "map",
    //     &MapView {
    //         background: MapBackground::new(MapProvider::OpenStreetMap),
    //         zoom: MapZoom::new(ZoomLevel(0)),
    //     },
    // );

    'recv: loop {
        match sdr.read_sync(&mut buf) {
            Ok(n) => {
                // let dump_start_pos = bitmap.stream_position().unwrap();
                // raw.write_all(buf[..n].as_ref())
                //     .expect("Unable to write to dump file");

                // if n < DEFAULT_BUF_LENGTH {
                //     info!("Short read ({n:#?}), samples lost, exiting!");
                //     break;
                // }
                // info!("Read {n} samples");

                let mut samples: Vec<f32> = Vec::with_capacity(n / 2);
                for c in buf[..n].chunks_exact(2) {
                    let i = (c[0] as f32 - 127.0) / 127.0;
                    let q = (c[1] as f32 - 127.0) / 127.0;

                    // let mag = ().sqrt
                    let mag = (i * i + q * q).sqrt();
                    // let mag = i.hypot(q);
                    samples.push(mag);
                    bitmap.write_all(&[(mag * 255.0) as u8; 3])?;
                }

                for samples in samples.windows(240) {
                    if check_preamble(samples) {
                        let data_raw = &samples[16..];
                        let avg_amp = data_raw.iter().sum::<f32>() / data_raw.len() as f32;
                        // if avg_amp < 0.1 {
                        //     continue;
                        // }
                        let mut data_bytes = pulses_to_bytes(&data_raw);
                        // if data_bytes[0] != 0x5d && data_bytes[0] != 0x8d {
                        //     continue;
                        // }
                        // let data_bytes = bits_to_bytes(&data_bits);
                        // print!("Preamble: ");
                        // print_magnitude_block(&samples[..16]);
                        // print!(" | Data: ");
                        // print_magnitude_block(&samples[16..16 + 80]);
                        // println!();

                        // println!(
                        //     "Preamble detected! avg amp {avg_amp:.2} {}",
                        //     data_bits
                        //         .iter()
                        //         .map(|b| if *b { 1 } else { 0 }.to_string())
                        //         .collect::<Vec<_>>()
                        //         .join("")
                        // );

                        // let mode_bits = &data_bits[0..5];
                        // let mode_num = bits_to_u32(mode_bits);
                        let downlink_format = data_bytes[0] >> 3;

                        let bits = if downlink_format <= 11 { 56 } else { 112 };
                        let crc_calculated = mode_s_checksum(&data_bytes, bits);
                        let crc_message = {
                            let end = bits / 8;
                            (data_bytes[end - 3] as u32) << 16
                                | (data_bytes[end - 2] as u32) << 8
                                | data_bytes[end - 1] as u32
                        };

                        if crc_calculated != crc_message {
                            if let Some(fix_pos) = fix_single_bit_error(&mut data_bytes, bits) {
                                warn!("Fixed bit at position {}", fix_pos);
                            } else {
                                // TODO(cohae): double bit error correction is quite slow. if we want to use it we will need to parallelize the program first
                                // if let Some(fix_pos) = fix_double_bit_error(&mut data_bytes, bits) {
                                //     warn!("Fixed 2 bits at position {}", fix_pos);
                                // } else {
                                //     // error!(
                                //     //     "CRC mismatch: calculated {crc_calculated:08X}, message {crc_message:08X}"
                                //     // );
                                continue;
                                // }
                            }
                        }
                        info!("CRC ok ({crc_calculated:08X})");
                        // info!("CRC: {crc_calculated:08X}, Message: {crc_message:08X}");
                        info!(
                            "*{}",
                            data_bytes
                                .iter()
                                .map(|&b| format!("{:02x}", b))
                                .collect::<Vec<_>>()
                                .join("")
                        );

                        let mode = match downlink_format {
                            0 => Some("Short air-air surveillance (ACAS)"),
                            4 => Some("Surveillance, altitude reply"),
                            5 => Some("Surveillance, identity reply"),
                            11 => Some("All-Call reply"),
                            16 => Some("Long air-air surveillance (ACAS)"),
                            17 => Some("Extended squitter"),
                            18 => Some("Extended squitter/non transponder"),
                            19 => Some("Military extended squitter"),
                            20 => Some("Comm-B, altitude reply"),
                            21 => Some("Comm-B, identity reply"),
                            // "Format number 24 is identified using only the first two bits, which must be 11 in binary. All following bits are used for encoding other information"
                            24..=31 => Some("Comm-D (ELM)"),
                            _ => None,
                        };
                        info!("  Mode: {mode:?} ({downlink_format})");
                        let craft = if matches!(downlink_format, 11 | 17 | 18) {
                            // println!(
                            //     ">>>>>>>>>>> Extended squitter detected, parsing ICAO address and CA..."
                            // );
                            // let ca = bits_to_u32(&data_bits[5..8]);
                            // let icao = bits_to_u32(&data_bits[8..32]);
                            let ca = data_bytes[0] & 0b111;
                            let icao = (data_bytes[1] as u32) << 16
                                | (data_bytes[2] as u32) << 8
                                | data_bytes[3] as u32;
                            info!("ICAO: {icao:06X}, DF: {downlink_format}, CA: {ca:02X}");
                            aircrafts
                                .entry(Icao::new(icao))
                                .or_insert_with(|| Aircraft::new(Icao::new(icao)))
                        } else {
                            warn!("Unhandled Mode S DF {downlink_format}");
                            continue;
                        };

                        if matches!(downlink_format, 17 | 18) {
                            let interrogator = (data_bytes[11] as u32) << 16
                                | (data_bytes[12] as u32) << 8
                                | data_bytes[13] as u32;

                            // interrogators.insert(interrogator);
                            // println!(
                            //     "{} Interrogators: {:X?}",
                            //     interrogators.len(),
                            //     interrogators
                            // );
                            // let interrogator = &data_bytes[11..14];
                            // println!(
                            //     "Interrogator ID: {}",
                            //     interrogator
                            //         .iter()
                            //         .map(|&b| format!("{b:02X}"))
                            //         .collect::<String>()
                            // );

                            // let message = &data_bytes[4..11];
                            let mut msg = BitReader::new(Cow::Borrowed(&data_bytes[4..11]));
                            // let message_type = message[0] >> 3;
                            let message_type = msg.read_bits(5);
                            let message_typename = match message_type {
                                1..=4 => "Aircraft identification",
                                5..=8 => "Surface position",
                                9..=18 => "Airborne position (w/Baro Altitude)",
                                19 => "Airborne velocities",
                                20..=22 => "Airborne position (w/GNSS Height)",
                                23..=27 => "Reserved",
                                28 => "Aircraft status",
                                29 => "Target state and status information",
                                31 => "Aircraft operation status",
                                _ => {
                                    error!("Unknown message type: {}", message_type);
                                    continue;
                                }
                            };

                            info!("ADS-B message '{message_typename}' ({message_type})");
                            info!(
                                "ADS-B message data: {}",
                                msg.data()
                                    .iter()
                                    .map(|b| format!("{:02X}", b))
                                    .collect::<Vec<_>>()
                                    .join("")
                            );

                            match message_type {
                                1..=4 => {
                                    let aircraft_type = msg.read_bits(3);
                                    let aircraft_category = match (message_type, aircraft_type) {
                                        (1, _) => "Reserved",
                                        (_, 0) => "No category information",
                                        (2, 1) => "Surface emergency vehicle",
                                        (2, 3) => "Surface service vehicle",
                                        (2, 4..=7) => "Ground obstruction",
                                        (3, 1) => "Glider, sailplane",
                                        (3, 2) => "Lighter-than-air",
                                        (3, 3) => "Parachutist, skydiver",
                                        (3, 4) => "Ultralight, hang-glider, paraglider",
                                        (3, 5) => "Reserved",
                                        (3, 6) => "Unmanned aerial vehicle",
                                        (3, 7) => "Space or transatmospheric vehicle",
                                        (4, 1) => "Light (less than 7000 kg)",
                                        (4, 2) => "Medium 1 (between 7000 kg and 34000 kg)",
                                        (4, 3) => "Medium 2 (between 34000 kg to 136000 kg)",
                                        (4, 4) => "High vortex aircraft",
                                        (4, 5) => "Heavy (larger than 136000 kg)",
                                        (4, 6) => {
                                            "High performance (>5 g acceleration) and high speed (>400 kt)"
                                        }
                                        (4, 7) => "Rotorcraft",
                                        _ => "<unknown wake vortex category>",
                                    };

                                    info!("  Aircraft category: {}", aircraft_category);

                                    let mut callsign = String::new();
                                    const AIS_LOOKUP: &str = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### ###############0123456789######";
                                    for _ in 0..8 {
                                        callsign.push(
                                            AIS_LOOKUP
                                                .chars()
                                                .nth(msg.read_bits(6) as usize)
                                                .unwrap(),
                                        );
                                    }

                                    callsign = callsign.trim_end_matches("#").trim().to_string();
                                    info!("  Callsign: {}", callsign);
                                    craft.callsign = Some(callsign.clone());

                                    // rec.log(
                                    //     format!("world/plane/{}", craft.icao),
                                    //     &rerun::TextDocument::from_markdown(format!(
                                    //         "**Callsign:** {callsign}"
                                    //     )),
                                    // )?;
                                    rec.log(
                                        "logs",
                                        &rerun::TextLog::new(format!(
                                            "Craft {} updated callsign to {callsign}",
                                            craft.icao
                                        ))
                                        .with_level(rerun::TextLogLevel::DEBUG),
                                    )?;
                                }
                                9..=18 | 20..=22 => {
                                    let surveillance_status = match msg.read_bits(2) {
                                        0 => "No condition",
                                        1 => "Permanent alert",
                                        2 => "Temporary alert",
                                        3 => "Special condition",
                                        _ => unreachable!(),
                                    };

                                    let single_antenna = msg.read_bit();
                                    let encoded_altitude = msg.read_bits(12);
                                    let time = msg.read_bits(1);
                                    let is_odd_frame = msg.read_bit();
                                    let encoded_latitude = msg.read_bits(17);
                                    let encoded_longitude = msg.read_bits(17);

                                    if is_odd_frame {
                                        craft.odd_cprlat = encoded_latitude;
                                        craft.odd_cprlon = encoded_longitude;
                                        craft.odd_cprtime = Instant::now();
                                    } else {
                                        craft.even_cprlat = encoded_latitude;
                                        craft.even_cprlon = encoded_longitude;
                                        craft.even_cprtime = Instant::now();
                                    }
                                    craft.update_latlong();

                                    let altitude = if (9..=18).contains(&message_type) {
                                        let q = (encoded_altitude & (1 << 8)) != 0;
                                        let alt_low = encoded_altitude & 0b1111;
                                        let alt_high = encoded_altitude >> 5;
                                        let alt_fixed = alt_high << 4 | alt_low;
                                        if q {
                                            Altitude::Feet(alt_fixed * 25 - 1000)
                                        } else {
                                            Altitude::Feet(gray_to_binary(alt_fixed) * 100)
                                        }
                                    } else {
                                        Altitude::Meters(encoded_altitude)
                                    };

                                    info!("  Surveillance status: {}", surveillance_status);
                                    info!("  Altitude: {:?}", altitude);
                                    info!("  Latitude: {} (not decoded)", encoded_latitude);
                                    info!("  Longitude: {} (not decoded)", encoded_longitude);
                                    info!("  Lat/Long: {:?}", craft.latlong());

                                    if let Some((lat, long)) = craft.latlong() {
                                        rec.log(
                                            format!("world/plane/{}", craft.icao),
                                            &rerun::GeoLineStrings::from_lat_lon([
                                                GeoLineString::from_iter(&craft.path),
                                            ]),
                                        )?;
                                        rec.log(
                                            format!("world/plane/{}", craft.icao),
                                            &rerun::GeoPoints::from_lat_lon(&craft.path)
                                                .with_radii(craft.path.iter().enumerate().map(
                                                    |(i, _)| {
                                                        if i == craft.path.len() - 1 {
                                                            Radius::new_ui_points(10.0)
                                                        } else {
                                                            Radius::new_ui_points(5.0)
                                                        }
                                                    },
                                                )),
                                        )?;
                                        // rec.log(
                                        //     "logs",
                                        //     &rerun::TextLog::new(format!(
                                        //         "Craft {} updated lat/long to {lat:.2}/{long:.2}",
                                        //         craft.icao
                                        //     ))
                                        //     .with_level(rerun::TextLogLevel::DEBUG),
                                        // )?;
                                    }
                                }
                                u => {
                                    error!("Unhandled ADS-B message type: {u}");
                                }
                            }
                        }

                        println!();
                    }
                }

                // let pulses = detect_mode_s_pulses(&samples, SAMPLE_RATE as f32, 1.5);
                // let mut bpulses = vec![false; samples.len()];
                // for &pulse in &pulses {
                //     bpulses[pulse] = true;
                // }
                // println!(
                //     "{}",
                //     bpulses
                //         .iter()
                //         .map(|b| (*b as u8).to_string())
                //         .collect::<Vec<_>>()
                //         .join("")
                // );

                // let mut recolor_samples = |range: Range<usize>, mask: [u8; 3]| {
                //     let save_pos = bitmap.stream_position().unwrap();
                //     bitmap
                //         .seek(std::io::SeekFrom::Start(
                //             dump_start_pos + (range.start * 3) as u64,
                //         ))
                //         .expect("Unable to seek in dump file");

                //     if let Some(samples) = samples.get(range) {
                //         for &val in samples {
                //             let b = (val * 255.0) as u8;
                //             bitmap
                //                 .write_all(&[mask[0] * b, mask[1] * b, mask[2] * b])
                //                 .expect("Unable to write preamble data to dump file");
                //         }
                //     }

                //     bitmap
                //         .seek(std::io::SeekFrom::Start(save_pos))
                //         .expect("Unable to seek back in dump file");
                // };

                // // Check for preamble
                // let mut skip_samples = 0;
                // for i in 0..(samples.len() - 16) {
                //     if skip_samples > 0 {
                //         skip_samples -= 1;
                //         continue;
                //     }

                //     if check_preamble(&samples[i..i + 17]) {
                //         let preamble_start = i + 1;
                //         let frame_start = preamble_start + 16;

                //         recolor_samples(preamble_start..preamble_start + 16, [0, 1, 0]);
                //         recolor_samples(frame_start..frame_start + 224, [1, 0, 0]);
                //         // let save_pos = dump.stream_position().unwrap();
                //         // dump.seek(std::io::SeekFrom::Start(
                //         //     dump_start_pos + (preamble_start * 3) as u64,
                //         // ))
                //         // .expect("Unable to seek in dump file");

                //         // for &val in preamble_data {
                //         //     dump.write_all(&[0, (val * 255.0) as u8, 0])
                //         //         .expect("Unable to write preamble data to dump file");
                //         // }

                //         // dump.seek(std::io::SeekFrom::Start(save_pos))
                //         //     .expect("Unable to seek back in dump file");

                //         // info!(
                //         //     "Preamble found at index {preamble_start} (y={} x={})",
                //         //     (preamble_start / 1024),
                //         //     (preamble_start % 1024),
                //         // );

                //         let frame_data = &samples[frame_start..frame_start + 224];
                //         let frame_pulses = samples_to_pulses(frame_data);
                //         info!(
                //             "Frame data: {:?}",
                //             frame_pulses
                //                 .iter()
                //                 .map(|b| (*b as u8).to_string())
                //                 .collect::<Vec<_>>()
                //                 .join("")
                //         );
                //         let mode_bits = &frame_pulses[0..5];
                //         let mode_num = pulses_to_bits(mode_bits);
                //         let mode = match mode_num {
                //             0 => Some("Short air-air surveillance (ACAS)"),
                //             4 => Some("Surveillance, altitude reply"),
                //             5 => Some("Surveillance, identity reply"),
                //             11 => Some("All-Call reply"),
                //             16 => Some("Long air-air surveillance (ACAS)"),
                //             17 => Some("Extended squitter"),
                //             18 => Some("Extended squitter/non transponder"),
                //             19 => Some("Military extended squitter"),
                //             20 => Some("Comm-B, altitude reply"),
                //             21 => Some("Comm-B, identity reply"),
                //             // "Format number 24 is identified using only the first two bits, which must be 11 in binary. All following bits are used for encoding other information"
                //             24..=31 => Some("Comm-D (ELM)"),
                //             _ => None,
                //         };

                //         if let Some(mode) = mode {
                //             info!("Mode: {mode} ({mode_num})");
                //         } else {
                //             error!("Unknown/invalid mode: {mode_num}");
                //         }

                //         if matches!(mode_num, 17 | 18) {
                //             println!(
                //                 ">>>>>>>>>>> Extended squitter detected, parsing ICAO address and CA..."
                //             );
                //             let ca = pulses_to_bits(&frame_pulses[5..8]);
                //             let icao = pulses_to_bits(&frame_pulses[8..32]);
                //             info!("ICAO: {icao:06X}, CA: {ca:02X}");
                //         }

                //         println!();

                //         skip_samples = 224; // Skip the next 224 samples (112 complex samples)
                //         // break 'recv;
                //     }
                // }

                // dump.write_all(&buf[..n])
                //     .expect("Unable to write to dump file");
            }
            Err(e) => {
                error!("Read error: {e:#?}");
            }
        }
    }
}

#[derive(Debug, Clone)]
enum Altitude {
    Feet(u32),
    Meters(u32),
}

impl Altitude {
    fn to_feet(&self) -> Self {
        match self {
            Self::Feet(feet) => Self::Feet(*feet),
            Self::Meters(meters) => Self::Feet((*meters as f32 * 3.28084) as u32),
        }
    }

    fn to_meters(&self) -> Self {
        match self {
            Self::Feet(feet) => Self::Meters((*feet as f32 / 3.28084) as u32),
            Self::Meters(meters) => Self::Meters(*meters),
        }
    }
}

// fn bits_to_bytes(bits: &[bool]) -> Vec<u8> {
//     let mut bytes = Vec::with_capacity(bits.len() / 8);

//     for c in bits.chunks(8) {
//         bytes.push(
//             (c[0] as u8) << 7
//                 | (c[1] as u8) << 6
//                 | (c[2] as u8) << 5
//                 | (c[3] as u8) << 4
//                 | (c[4] as u8) << 3
//                 | (c[5] as u8) << 2
//                 | (c[6] as u8) << 1
//                 | (c[7] as u8),
//         );
//     }

//     bytes
// }

/// Convert a Gray code to binary.
fn gray_to_binary(mut gray: u32) -> u32 {
    let mut binary = 0;
    let mut mask = 1;

    for _ in 0..32 {
        let bit = gray & mask;
        binary |= bit;
        gray ^= bit;
        mask <<= 1;
    }

    binary
}

fn print_magnitude_block(mag: &[f32]) {
    for m in mag {
        let m8 = (m * 255.0) as u8;
        print!("\x1b[38;2;{m8};{m8};{m8}m");
        print!("██");
        print!("\x1b[0m");
    }
}

fn check_preamble(m: &[f32]) -> bool {
    if m.len() < 16 {
        return false;
    }

    // Preamble 1010000101000000
    if !(m[0] > m[0 + 1]
        && m[0 + 1] < m[0 + 2]
        && m[0 + 2] > m[0 + 3]
        && m[0 + 3] < m[0]
        && m[0 + 4] < m[0]
        && m[0 + 5] < m[0]
        && m[0 + 6] < m[0]
        && m[0 + 7] > m[0 + 8]
        && m[0 + 8] < m[0 + 9]
        && m[0 + 9] > m[0 + 6])
    {
        return false;
    }

    // let hi0 = magnitude[0]; // 1
    // let lo1 = magnitude[1]; // 0
    // let hi2 = magnitude[2]; // 1
    // let lo3 = magnitude[3]; // 0
    // let lo4 = magnitude[4]; // 0
    // let lo5 = magnitude[5]; // 0
    // let lo6 = magnitude[6]; // 0
    // let hi7 = magnitude[7]; // 1
    // let lo8 = magnitude[8]; // 0
    // let hi9 = magnitude[9]; // 1
    // let lo10 = magnitude[10]; // 0
    // let lo11 = magnitude[11]; // 0
    // let lo12 = magnitude[12]; // 0
    // let lo13 = magnitude[13]; // 0
    // let lo14 = magnitude[14]; // 0
    // let lo15 = magnitude[15]; // 0

    // let avg_hi = (hi0 + hi2 + hi7 + hi9) / 4.0;
    // let avg_lo =
    //     (lo1 + lo3 + lo4 + lo5 + lo6 + lo8 + lo10 + lo11 + lo12 + lo13 + lo14 + lo15) / 12.0;

    // if avg_hi < avg_lo {
    //     return false;
    // }

    // // Ensure all low samples are below the average high sample
    // if lo1 > avg_hi
    //     || lo3 > avg_hi
    //     || lo4 > avg_hi
    //     || lo5 > avg_hi
    //     || lo6 > avg_hi
    //     || lo8 > avg_hi
    //     || lo10 > avg_hi
    //     || lo11 > avg_hi
    //     || lo12 > avg_hi
    //     || lo13 > avg_hi
    //     || lo14 > avg_hi
    //     || lo15 > avg_hi
    // {
    //     return false;
    // }

    // // Ensure all high samples are above the average low sample
    // if hi0 < avg_lo || hi2 < avg_lo || hi7 < avg_lo || hi9 < avg_lo {
    //     return false;
    // }

    // // Check peaks/valleys
    // if hi0 < lo1 || lo1 > hi2 || hi2 < lo3 || lo6 > hi7 || hi7 < lo8 || lo8 > hi9 || hi9 < lo10 {
    //     return false;
    // }

    true
}

fn pulses_to_bytes(magnitudes: &[f32]) -> Vec<u8> {
    let mut bits = vec![false; magnitudes.len() / 2];

    for (i, pulse_set) in magnitudes.chunks_exact(2).enumerate() {
        let low = pulse_set[0];
        let high = pulse_set[1];

        // let delta = (low - high).abs();
        // if i > 0 && delta < 0.02 {
        //     bits[i] = bits[i - 1];
        // } else if low == high {
        //     bits[i] = false;
        // } else
        if low > high {
            bits[i] = true;
        } else {
            bits[i] = false;
        }

        // bits.push(pulse_set[0] > pulse_set[1]);
    }

    let mut bytes = Vec::with_capacity(bits.len() / 8);

    for c in bits.chunks_exact(8) {
        bytes.push(
            (c[0] as u8) << 7
                | (c[1] as u8) << 6
                | (c[2] as u8) << 5
                | (c[3] as u8) << 4
                | (c[4] as u8) << 3
                | (c[5] as u8) << 2
                | (c[6] as u8) << 1
                | (c[7] as u8),
        );
    }

    bytes
}

fn bits_to_u32(bits: &[bool]) -> u32 {
    let mut value = 0;
    for (i, bit) in bits.iter().enumerate() {
        if *bit {
            value |= 1 << i;
        }
    }
    value
}

const MODE_S_CHECKSUM_LUT: [u32; 112] = [
    0x3935ea, 0x1c9af5, 0xf1b77e, 0x78dbbf, 0xc397db, 0x9e31e9, 0xb0e2f0, 0x587178, 0x2c38bc,
    0x161c5e, 0x0b0e2f, 0xfa7d13, 0x82c48d, 0xbe9842, 0x5f4c21, 0xd05c14, 0x682e0a, 0x341705,
    0xe5f186, 0x72f8c3, 0xc68665, 0x9cb936, 0x4e5c9b, 0xd8d449, 0x939020, 0x49c810, 0x24e408,
    0x127204, 0x093902, 0x049c81, 0xfdb444, 0x7eda22, 0x3f6d11, 0xe04c8c, 0x702646, 0x381323,
    0xe3f395, 0x8e03ce, 0x4701e7, 0xdc7af7, 0x91c77f, 0xb719bb, 0xa476d9, 0xadc168, 0x56e0b4,
    0x2b705a, 0x15b82d, 0xf52612, 0x7a9309, 0xc2b380, 0x6159c0, 0x30ace0, 0x185670, 0x0c2b38,
    0x06159c, 0x030ace, 0x018567, 0xff38b7, 0x80665f, 0xbfc92b, 0xa01e91, 0xaff54c, 0x57faa6,
    0x2bfd53, 0xea04ad, 0x8af852, 0x457c29, 0xdd4410, 0x6ea208, 0x375104, 0x1ba882, 0x0dd441,
    0xf91024, 0x7c8812, 0x3e4409, 0xe0d800, 0x706c00, 0x383600, 0x1c1b00, 0x0e0d80, 0x0706c0,
    0x038360, 0x01c1b0, 0x00e0d8, 0x00706c, 0x003836, 0x001c1b, 0xfff409, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000,
];

fn mode_s_checksum(data: &[u8], bits: usize) -> u32 {
    let mut crc = 0u32;
    let offset = if bits == 112 { 0 } else { 112 - 56 };

    for j in 0..bits {
        let byte = j / 8;
        let bit = j % 8;
        let mask = 1 << (7 - bit);
        if data[byte] & mask != 0 {
            crc ^= MODE_S_CHECKSUM_LUT[j + offset];
        }
    }

    crc
}

fn fix_single_bit_error(data: &mut [u8], bits: usize) -> Option<usize> {
    let mut corrected = [0u8; 112 / 8];

    for j in 0..bits {
        let byte = j / 8;
        let bit = j % 8;
        let bitmask = 1 << (7 - bit);

        corrected.copy_from_slice(data);
        corrected[byte] ^= bitmask;
        let end = bits / 8;
        let crc1 = (corrected[end - 3] as u32) << 16
            | (corrected[end - 2] as u32) << 8
            | corrected[end - 1] as u32;
        let crc2 = mode_s_checksum(&corrected, bits);
        if crc1 == crc2 {
            data[0..112 / 8].copy_from_slice(&corrected);
            return Some(j);
        }
    }

    None
}

fn fix_double_bit_error(data: &mut [u8], bits: usize) -> Option<usize> {
    let mut corrected = [0u8; 112 / 8];

    for j in 0..bits {
        let byte1 = j / 8;
        let bit1 = j % 8;
        let bitmask1 = 1 << (7 - bit1);

        for i in j + 1..bits {
            let byte2 = i / 8;
            let bit2 = i % 8;
            let bitmask2 = 1 << (7 - bit2);

            corrected.copy_from_slice(data);
            corrected[byte1] ^= bitmask1;
            corrected[byte2] ^= bitmask2;

            let end = bits / 8;
            let crc1 = (corrected[end - 3] as u32) << 16
                | (corrected[end - 2] as u32) << 8
                | corrected[end - 1] as u32;
            let crc2 = mode_s_checksum(&corrected, bits);
            if crc1 == crc2 {
                data[0..112 / 8].copy_from_slice(&corrected);
                return Some(j);
            }
        }
    }

    None
}
