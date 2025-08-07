use std::f64::consts::PI;

use crate::aircraft::Aircraft;

pub fn decode_cpr(a: &Aircraft) -> Option<(f64, f64)> {
    let air_dlat0 = 360.0 / 60.0;
    let air_dlat1 = 360.0 / 59.0;

    let lat0 = a.even_cprlat as f64;
    let lat1 = a.odd_cprlat as f64;

    let lon0 = a.even_cprlon as f64;
    let lon1 = a.odd_cprlon as f64;

    let j = (((59.0 * lat0 - 60.0 * lat1) / 131072.0) + 0.5).floor() as i32;
    let mut rlat0 = air_dlat0 * (cpr_mod(j, 60) as f64 + lat0 / 131072.0);
    let mut rlat1 = air_dlat1 * (cpr_mod(j, 59) as f64 + lat1 / 131072.0);

    if rlat0 >= 270.0 {
        rlat0 -= 360.0;
    }
    if rlat1 >= 270.0 {
        rlat1 -= 360.0;
    }

    if cpr_nl(rlat0) != cpr_nl(rlat1) {
        return None;
    }

    let (rlat, lon, is_odd) = if a.even_cprtime > a.odd_cprtime {
        (rlat0, lon0, false)
    } else {
        (rlat1, lon1, true)
    };

    let ni = cpr_n(rlat, is_odd);
    let m = ((((lon0 * (cpr_nl(rlat) - 1) as f64) - (lon1 * cpr_nl(rlat) as f64)) / 131072.0) + 0.5)
        .floor() as i32;

    let lat = rlat;
    let mut lon = cpr_dlon(rlat, is_odd) * (cpr_mod(m, ni) as f64 + lon / 131072.0);

    if lon > 180.0 {
        lon -= 360.0;
    }

    // if a.even_cprtime > a.odd_cprtime {
    //     println!("Even {lat:.2} {lon:.2} (HACK +3 lon)");
    // } else {
    //     println!("Odd {lat:.2} {lon:.2}");
    // }

    Some((lat, lon))
}

fn cpr_mod(a: i32, b: i32) -> i32 {
    let mut res = a % b;
    if res < 0 {
        res += b;
    }
    res
}

fn cpr_n(lat: f64, is_odd: bool) -> i32 {
    let mut nl = cpr_nl(lat) - is_odd as i32;
    if nl < 1 {
        nl = 1;
    }
    nl
}

fn cpr_dlon(lat: f64, is_odd: bool) -> f64 {
    360.0 / cpr_n(lat, is_odd) as f64
}

/// Calculate the longitude zone number for the given latitude
fn cpr_nl(lat: f64) -> i32 {
    // The number of longitude zones
    const NZ: f64 = 15.0;

    let a = 1.0 - (PI / (2.0 * NZ)).cos();
    let b = lat.to_radians().cos().powi(2);
    let c = 1.0 - a / b;

    if c.abs() > 1.0 {
        return 1;
    }

    ((2.0 * PI) / c.acos()).floor() as i32

    // if lat < 0.0 {
    //     lat = -lat;
    // }

    // match lat {
    //     _ if (lat < 10.47047130) => 59,
    //     _ if (lat < 14.82817437) => 58,
    //     _ if (lat < 18.18626357) => 57,
    //     _ if (lat < 21.02939493) => 56,
    //     _ if (lat < 23.54504487) => 55,
    //     _ if (lat < 25.82924707) => 54,
    //     _ if (lat < 27.93898710) => 53,
    //     _ if (lat < 29.91135686) => 52,
    //     _ if (lat < 31.77209708) => 51,
    //     _ if (lat < 33.53993436) => 50,
    //     _ if (lat < 35.22899598) => 49,
    //     _ if (lat < 36.85025108) => 48,
    //     _ if (lat < 38.41241892) => 47,
    //     _ if (lat < 39.92256684) => 46,
    //     _ if (lat < 41.38651832) => 45,
    //     _ if (lat < 42.80914012) => 44,
    //     _ if (lat < 44.19454951) => 43,
    //     _ if (lat < 45.54626723) => 42,
    //     _ if (lat < 46.86733252) => 41,
    //     _ if (lat < 48.16039128) => 40,
    //     _ if (lat < 49.42776439) => 39,
    //     _ if (lat < 50.67150166) => 38,
    //     _ if (lat < 51.89342469) => 37,
    //     _ if (lat < 53.09516153) => 36,
    //     _ if (lat < 54.27817472) => 35,
    //     _ if (lat < 55.44378444) => 34,
    //     _ if (lat < 56.59318756) => 33,
    //     _ if (lat < 57.72747354) => 32,
    //     _ if (lat < 58.84763776) => 31,
    //     _ if (lat < 59.95459277) => 30,
    //     _ if (lat < 61.04917774) => 29,
    //     _ if (lat < 62.13216659) => 28,
    //     _ if (lat < 63.20427479) => 27,
    //     _ if (lat < 64.26616523) => 26,
    //     _ if (lat < 65.31845310) => 25,
    //     _ if (lat < 66.36171008) => 24,
    //     _ if (lat < 67.39646774) => 23,
    //     _ if (lat < 68.42322022) => 22,
    //     _ if (lat < 69.44242631) => 21,
    //     _ if (lat < 70.45451075) => 20,
    //     _ if (lat < 71.45986473) => 19,
    //     _ if (lat < 72.45884545) => 18,
    //     _ if (lat < 73.45177442) => 17,
    //     _ if (lat < 74.43893416) => 16,
    //     _ if (lat < 75.42056257) => 15,
    //     _ if (lat < 76.39684391) => 14,
    //     _ if (lat < 77.36789461) => 13,
    //     _ if (lat < 78.33374083) => 12,
    //     _ if (lat < 79.29428225) => 11,
    //     _ if (lat < 80.24923213) => 10,
    //     _ if (lat < 81.19801349) => 9,
    //     _ if (lat < 82.13956981) => 8,
    //     _ if (lat < 83.07199445) => 7,
    //     _ if (lat < 83.99173563) => 6,
    //     _ if (lat < 84.89166191) => 5,
    //     _ if (lat < 85.75541621) => 4,
    //     _ if (lat < 86.53536998) => 3,
    //     _ if (lat < 87.00000000) => 2,
    //     _ => 1,
    // }

    // if (lat < 10.47047130) return 59;
    // if (lat < 14.82817437) return 58;
    // if (lat < 18.18626357) return 57;
    // if (lat < 21.02939493) return 56;
    // if (lat < 23.54504487) return 55;
    // if (lat < 25.82924707) return 54;
    // if (lat < 27.93898710) return 53;
    // if (lat < 29.91135686) return 52;
    // if (lat < 31.77209708) return 51;
    // if (lat < 33.53993436) return 50;
    // if (lat < 35.22899598) return 49;
    // if (lat < 36.85025108) return 48;
    // if (lat < 38.41241892) return 47;
    // if (lat < 39.92256684) return 46;
    // if (lat < 41.38651832) return 45;
    // if (lat < 42.80914012) return 44;
    // if (lat < 44.19454951) return 43;
    // if (lat < 45.54626723) return 42;
    // if (lat < 46.86733252) return 41;
    // if (lat < 48.16039128) return 40;
    // if (lat < 49.42776439) return 39;
    // if (lat < 50.67150166) return 38;
    // if (lat < 51.89342469) return 37;
    // if (lat < 53.09516153) return 36;
    // if (lat < 54.27817472) return 35;
    // if (lat < 55.44378444) return 34;
    // if (lat < 56.59318756) return 33;
    // if (lat < 57.72747354) return 32;
    // if (lat < 58.84763776) return 31;
    // if (lat < 59.95459277) return 30;
    // if (lat < 61.04917774) return 29;
    // if (lat < 62.13216659) return 28;
    // if (lat < 63.20427479) return 27;
    // if (lat < 64.26616523) return 26;
    // if (lat < 65.31845310) return 25;
    // if (lat < 66.36171008) return 24;
    // if (lat < 67.39646774) return 23;
    // if (lat < 68.42322022) return 22;
    // if (lat < 69.44242631) return 21;
    // if (lat < 70.45451075) return 20;
    // if (lat < 71.45986473) return 19;
    // if (lat < 72.45884545) return 18;
    // if (lat < 73.45177442) return 17;
    // if (lat < 74.43893416) return 16;
    // if (lat < 75.42056257) return 15;
    // if (lat < 76.39684391) return 14;
    // if (lat < 77.36789461) return 13;
    // if (lat < 78.33374083) return 12;
    // if (lat < 79.29428225) return 11;
    // if (lat < 80.24923213) return 10;
    // if (lat < 81.19801349) return 9;
    // if (lat < 82.13956981) return 8;
    // if (lat < 83.07199445) return 7;
    // if (lat < 83.99173563) return 6;
    // if (lat < 84.89166191) return 5;
    // if (lat < 85.75541621) return 4;
    // if (lat < 86.53536998) return 3;
    // if (lat < 87.00000000) return 2;
    // else return 1;
}

#[test]
fn test_cpr_nl() {
    assert_eq!(cpr_nl(5.0), 59);
    assert_eq!(cpr_nl(50.0), 38);
    assert_eq!(cpr_nl(86.6), 2);
    assert_eq!(cpr_nl(88.0), 1);
}
