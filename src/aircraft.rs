use std::{
    fmt::{Debug, Display},
    time::Instant,
};

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

    pub latitude: f64,
    pub longitude: f64,

    pub path: Vec<(f64, f64)>,
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
            latitude: 0.0,
            longitude: 0.0,
            path: Vec::new(),
        }
    }

    pub fn latlong(&self) -> Option<(f64, f64)> {
        (self.latitude != 0.0 && self.longitude != 0.0).then(|| (self.latitude, self.longitude))
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
            self.path.push(latlon);
        }
    }
}
