use std::{
    fmt::{Debug, Display},
    time::Instant,
};

use glam::DVec2;
use rerun::{Radius, components::GeoLineString};

use crate::cpr;

#[derive(Clone, Hash, PartialEq, Eq)]
pub struct Icao(u32);

impl Icao {
    pub fn new(value: u32) -> Self {
        debug_assert!(value <= 0xFF_FFFF, "ICAO code must be a 24-bit number");
        Icao(value)
    }
}

impl Display for Icao {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:06X}", self.0)
    }
}

impl Debug for Icao {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Icao({:06X})", self.0)
    }
}

#[derive(Debug, Clone)]
pub struct Aircraft {
    pub icao: Icao,
    pub callsign: Option<String>,

    pub even_cprlat: u32,
    pub even_cprlon: u32,
    pub even_cprtime: Instant,
    pub odd_cprlat: u32,
    pub odd_cprlon: u32,
    pub odd_cprtime: Instant,

    pub altitude: Option<Altitude>,
    pub latitude: f64,
    pub longitude: f64,
    pub last_pos_update: Instant,

    pub latitude_interpolated: f64,
    pub longitude_interpolated: f64,

    pub path: Vec<(f64, f64)>,

    pub velocity_kts: Option<DVec2>,
}

impl Aircraft {
    pub fn new(icao: Icao) -> Self {
        Aircraft {
            icao,
            callsign: None,
            even_cprlat: 0,
            even_cprlon: 0,
            even_cprtime: Instant::now(),
            odd_cprlat: 0,
            odd_cprlon: 0,
            odd_cprtime: Instant::now(),
            altitude: None,
            latitude: 0.0,
            longitude: 0.0,
            last_pos_update: Instant::now(),
            latitude_interpolated: 0.0,
            longitude_interpolated: 0.0,
            path: Vec::new(),
            velocity_kts: None,
        }
    }

    pub fn latlong(&self) -> Option<(f64, f64)> {
        (self.latitude != 0.0 && self.longitude != 0.0).then_some((self.latitude, self.longitude))
    }

    pub fn update_latlong(&mut self) {
        let last_even = self.even_cprtime.elapsed();
        let last_odd = self.odd_cprtime.elapsed();
        let delta = (last_even.as_millis() as i128 - last_odd.as_millis() as i128).abs();

        // Lat/long updates more than 10 seconds apart should not be trusted
        if delta > 10_000 {
            return;
        }

        if self.even_cprlat == 0
            || self.even_cprlon == 0
            || self.odd_cprlat == 0
            || self.odd_cprlon == 0
        {
            return;
        }

        if let Some(latlon) = cpr::decode_cpr(self) {
            (self.latitude, self.longitude) = latlon;
            (self.latitude_interpolated, self.longitude_interpolated) = latlon;
            self.path.push(latlon);
            self.last_pos_update = Instant::now();
        }
    }

    pub fn speed_kts(&self) -> Option<f64> {
        self.velocity_kts.map(|v| v.length())
    }

    pub fn heading(&self) -> Option<f64> {
        self.velocity_kts.map(|v| {
            let mut a = v.x.atan2(v.y).to_degrees();
            if a < 0.0 {
                a += 360.0;
            }
            a
        })
    }

    pub fn log_rerun(&self, rec: &rerun::RecordingStream) -> anyhow::Result<()> {
        macro_rules! path_with_prediction {
            () => {
                self.path
                    .iter()
                    .chain([(self.latitude_interpolated, self.longitude_interpolated)].iter())
            };
        }

        let ent_path = format!("world/plane/{}", self.icao);
        rec.log(
            ent_path.clone(),
            &rerun::GeoLineStrings::from_lat_lon([GeoLineString::from_iter(
                path_with_prediction!(),
            )]),
        )?;
        rec.log(
            ent_path.clone(),
            &rerun::GeoPoints::from_lat_lon(path_with_prediction!()).with_radii(
                path_with_prediction!().enumerate().map(|(i, _)| {
                    if i == self.path.len() {
                        Radius::new_ui_points(10.0)
                    } else {
                        Radius::new_ui_points(5.0)
                    }
                }),
            ),
        )?;

        let speed_kts = self
            .speed_kts()
            .map(|s| format!("{s:.2}"))
            .unwrap_or("pending".to_string());

        let heading = self
            .heading()
            .map(|h| format!("{h:.0} deg"))
            .unwrap_or("pending".to_string());

        let data = rerun::AnyValues::default()
            .with_component::<rerun::components::Text>(
                "callsign",
                vec![
                    self.callsign
                        .clone()
                        .unwrap_or_else(|| "pending".to_string()),
                ],
            )
            .with_component::<rerun::components::Text>("speed_kts", vec![speed_kts])
            .with_component::<rerun::components::Text>("heading", vec![heading])
            .with_component::<rerun::components::LatLon>(
                "latlong",
                vec![(self.latitude, self.longitude)],
            );
        rec.log(ent_path, &data)?;

        Ok(())
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Altitude {
    Feet(u32),
    Meters(u32),
}

impl Altitude {
    pub fn to_feet(self) -> Self {
        match self {
            Self::Feet(feet) => Self::Feet(feet),
            Self::Meters(meters) => Self::Feet((meters as f32 * 3.28084) as u32),
        }
    }

    pub fn to_meters(self) -> Self {
        match self {
            Self::Feet(feet) => Self::Meters((feet as f32 / 3.28084) as u32),
            Self::Meters(meters) => Self::Meters(meters),
        }
    }
}
