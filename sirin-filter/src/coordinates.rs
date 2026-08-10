use nalgebra::{SMatrix, SVector};

pub type Vec3d = SVector<f64, 3>;
pub type Mat3d = SMatrix<f64, 3, 3>;

const WGS84_A_M: f64 = 6_378_137.0;
const WGS84_E2: f64 = 6.694_379_990_14e-3;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeodeticPosition {
    pub latitude_rad: f64,
    pub longitude_rad: f64,
    pub ellipsoid_height_m: f64,
}

#[derive(Debug, Clone)]
pub struct LocalNedOrigin {
    pub geodetic: GeodeticPosition,
    pub ecef_m: Vec3d,
    ecef_to_ned: Mat3d,
}

impl LocalNedOrigin {
    pub fn new(geodetic: GeodeticPosition) -> Self {
        let sin_lat = libm::sin(geodetic.latitude_rad);
        let cos_lat = libm::cos(geodetic.latitude_rad);
        let sin_lon = libm::sin(geodetic.longitude_rad);
        let cos_lon = libm::cos(geodetic.longitude_rad);

        Self {
            geodetic,
            ecef_m: geodetic_to_ecef(&geodetic),
            ecef_to_ned: Mat3d::new(
                -sin_lat * cos_lon,
                -sin_lat * sin_lon,
                cos_lat,
                -sin_lon,
                cos_lon,
                0.0,
                -cos_lat * cos_lon,
                -cos_lat * sin_lon,
                -sin_lat,
            ),
        }
    }

    pub fn ecef_to_ned(&self, ecef_m: &Vec3d) -> Vec3d {
        self.ecef_to_ned * (ecef_m - self.ecef_m)
    }

    pub fn geodetic_to_ned(&self, geodetic: &GeodeticPosition) -> Vec3d {
        self.ecef_to_ned(&geodetic_to_ecef(geodetic))
    }
}

pub fn geodetic_to_ecef(geodetic: &GeodeticPosition) -> Vec3d {
    let sin_lat = libm::sin(geodetic.latitude_rad);
    let cos_lat = libm::cos(geodetic.latitude_rad);
    let sin_lon = libm::sin(geodetic.longitude_rad);
    let cos_lon = libm::cos(geodetic.longitude_rad);
    let prime_vertical_radius =
        WGS84_A_M / libm::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);

    Vec3d::new(
        (prime_vertical_radius + geodetic.ellipsoid_height_m) * cos_lat * cos_lon,
        (prime_vertical_radius + geodetic.ellipsoid_height_m) * cos_lat * sin_lon,
        (prime_vertical_radius * (1.0 - WGS84_E2) + geodetic.ellipsoid_height_m)
            * sin_lat,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn equator_prime_meridian_matches_wgs84_axis() {
        let ecef = geodetic_to_ecef(&GeodeticPosition {
            latitude_rad: 0.0,
            longitude_rad: 0.0,
            ellipsoid_height_m: 0.0,
        });
        assert!((ecef.x - WGS84_A_M).abs() < 1.0e-6);
        assert!(ecef.y.abs() < 1.0e-9);
        assert!(ecef.z.abs() < 1.0e-9);
    }

    #[test]
    fn increased_height_is_negative_down() {
        let origin_position = GeodeticPosition {
            latitude_rad: 0.0,
            longitude_rad: 0.0,
            ellipsoid_height_m: 10.0,
        };
        let origin = LocalNedOrigin::new(origin_position);
        let point = GeodeticPosition {
            ellipsoid_height_m: 110.0,
            ..origin_position
        };
        let ned = origin.geodetic_to_ned(&point);
        assert!(ned.x.abs() < 1.0e-6);
        assert!(ned.y.abs() < 1.0e-6);
        assert!((ned.z + 100.0).abs() < 1.0e-6);
    }
}
