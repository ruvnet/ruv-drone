//! Canonical, deterministic representation of drone advisory telemetry.
//!
//! Floating-point values never cross the LatentMesh boundary directly. They
//! are validated and converted to signed Q16.16 values before hashing or
//! signing. The schema IDs below are wire ABI and must never be renumbered.

use crate::{
    failsafe::FailSafeState,
    types::{DroneState, NodeId, Position3D, Velocity3D},
};
use latentmesh_air_core::{CriticalState, SymbolValue};

/// Version of the deterministic advisory telemetry schema.
pub const CRITICAL_STATE_SCHEMA_VERSION: u64 = 1;

pub const FIELD_SCHEMA_VERSION: u16 = 0x0001;
pub const FIELD_NODE_ID: u16 = 0x0010;
pub const FIELD_POSITION_NORTH_M: u16 = 0x0100;
pub const FIELD_POSITION_EAST_M: u16 = 0x0101;
pub const FIELD_POSITION_DOWN_M: u16 = 0x0102;
pub const FIELD_VELOCITY_NORTH_MPS: u16 = 0x0200;
pub const FIELD_VELOCITY_EAST_MPS: u16 = 0x0201;
pub const FIELD_VELOCITY_DOWN_MPS: u16 = 0x0202;
pub const FIELD_HEADING_RAD: u16 = 0x0300;
pub const FIELD_ALTITUDE_AGL_M: u16 = 0x0301;
pub const FIELD_BATTERY_PCT: u16 = 0x0400;
pub const FIELD_LINK_QUALITY: u16 = 0x0401;
pub const FIELD_TIMESTAMP_MS: u16 = 0x0402;
pub const FIELD_FAILSAFE_STATE: u16 = 0x0403;

const REQUIRED_FIELDS: [u16; 14] = [
    FIELD_SCHEMA_VERSION,
    FIELD_NODE_ID,
    FIELD_POSITION_NORTH_M,
    FIELD_POSITION_EAST_M,
    FIELD_POSITION_DOWN_M,
    FIELD_VELOCITY_NORTH_MPS,
    FIELD_VELOCITY_EAST_MPS,
    FIELD_VELOCITY_DOWN_MPS,
    FIELD_HEADING_RAD,
    FIELD_ALTITUDE_AGL_M,
    FIELD_BATTERY_PCT,
    FIELD_LINK_QUALITY,
    FIELD_TIMESTAMP_MS,
    FIELD_FAILSAFE_STATE,
];

const Q16_16_SCALE: f64 = 65_536.0;
const Q16_16_MIN: f64 = i32::MIN as f64 / Q16_16_SCALE;
const Q16_16_MAX: f64 = i32::MAX as f64 / Q16_16_SCALE;

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum StateError {
    #[error("field {field} is not finite")]
    NonFinite { field: &'static str },
    #[error("field {field} is outside [{minimum}, {maximum}]")]
    OutOfRange {
        field: &'static str,
        minimum: &'static str,
        maximum: &'static str,
    },
    #[error("critical field 0x{field_id:04x} is missing")]
    MissingField { field_id: u16 },
    #[error("critical field 0x{field_id:04x} has the wrong type")]
    WrongType { field_id: u16 },
    #[error("critical field 0x{field_id:04x} is unknown to schema version 1")]
    UnknownField { field_id: u16 },
    #[error("unsupported critical-state schema version {0}")]
    UnsupportedSchema(u64),
    #[error("unknown fail-safe state code {0}")]
    UnknownFailsafe(u64),
    #[error("LatentMesh state rejected the value: {0}")]
    Air(#[from] latentmesh_air_core::AirError),
}

/// A validated peer snapshot reconstructed exclusively from deterministic
/// symbols. Learned residuals are deliberately absent.
///
/// This type is observational and non-authoritative. It must not be inserted
/// into `MeshTopology`, the orchestrator's authoritative `peer_states`, or fed
/// directly to collision avoidance, geofence, failsafe, or flight control.
#[derive(Clone, Debug)]
pub struct AdvisoryPeerSnapshot {
    pub drone: DroneState,
    pub failsafe: FailSafeState,
}

/// Convert local telemetry to the canonical LatentMesh deterministic schema.
pub fn to_critical_state(
    drone: &DroneState,
    failsafe: &FailSafeState,
) -> Result<CriticalState, StateError> {
    validate_domain_ranges(drone)?;

    let mut state = CriticalState::new();
    state.set(
        FIELD_SCHEMA_VERSION,
        SymbolValue::U64(CRITICAL_STATE_SCHEMA_VERSION),
    )?;
    state.set(FIELD_NODE_ID, SymbolValue::U64(u64::from(drone.id.0)))?;
    set_q16(
        &mut state,
        FIELD_POSITION_NORTH_M,
        "position.x",
        drone.position.x,
    )?;
    set_q16(
        &mut state,
        FIELD_POSITION_EAST_M,
        "position.y",
        drone.position.y,
    )?;
    set_q16(
        &mut state,
        FIELD_POSITION_DOWN_M,
        "position.z",
        drone.position.z,
    )?;
    set_q16(
        &mut state,
        FIELD_VELOCITY_NORTH_MPS,
        "velocity.vx",
        drone.velocity.vx,
    )?;
    set_q16(
        &mut state,
        FIELD_VELOCITY_EAST_MPS,
        "velocity.vy",
        drone.velocity.vy,
    )?;
    set_q16(
        &mut state,
        FIELD_VELOCITY_DOWN_MPS,
        "velocity.vz",
        drone.velocity.vz,
    )?;
    set_q16(
        &mut state,
        FIELD_HEADING_RAD,
        "heading_rad",
        drone.heading_rad,
    )?;
    set_q16(
        &mut state,
        FIELD_ALTITUDE_AGL_M,
        "altitude_agl_m",
        drone.altitude_agl_m,
    )?;
    set_q16(
        &mut state,
        FIELD_BATTERY_PCT,
        "battery_pct",
        f64::from(drone.battery_pct),
    )?;
    set_q16(
        &mut state,
        FIELD_LINK_QUALITY,
        "link_quality",
        f64::from(drone.link_quality),
    )?;
    state.set(FIELD_TIMESTAMP_MS, SymbolValue::U64(drone.timestamp_ms))?;
    state.set(
        FIELD_FAILSAFE_STATE,
        SymbolValue::U64(failsafe_code(failsafe)),
    )?;
    Ok(state)
}

/// Reconstruct and validate a peer's complete advisory telemetry snapshot.
pub fn from_critical_state(state: &CriticalState) -> Result<AdvisoryPeerSnapshot, StateError> {
    validate_field_set(state)?;
    let schema = required_u64(state, FIELD_SCHEMA_VERSION)?;
    if schema != CRITICAL_STATE_SCHEMA_VERSION {
        return Err(StateError::UnsupportedSchema(schema));
    }

    let node = required_u64(state, FIELD_NODE_ID)?;
    let node = u32::try_from(node).map_err(|_| StateError::OutOfRange {
        field: "node_id",
        minimum: "0",
        maximum: "u32::MAX",
    })?;
    let failsafe = failsafe_from_code(required_u64(state, FIELD_FAILSAFE_STATE)?)?;
    let drone = DroneState {
        id: NodeId(node),
        position: Position3D {
            x: required_q16(state, FIELD_POSITION_NORTH_M)?,
            y: required_q16(state, FIELD_POSITION_EAST_M)?,
            z: required_q16(state, FIELD_POSITION_DOWN_M)?,
        },
        velocity: Velocity3D {
            vx: required_q16(state, FIELD_VELOCITY_NORTH_MPS)?,
            vy: required_q16(state, FIELD_VELOCITY_EAST_MPS)?,
            vz: required_q16(state, FIELD_VELOCITY_DOWN_MPS)?,
        },
        heading_rad: required_q16(state, FIELD_HEADING_RAD)?,
        altitude_agl_m: required_q16(state, FIELD_ALTITUDE_AGL_M)?,
        battery_pct: required_q16(state, FIELD_BATTERY_PCT)? as f32,
        link_quality: required_q16(state, FIELD_LINK_QUALITY)? as f32,
        timestamp_ms: required_u64(state, FIELD_TIMESTAMP_MS)?,
    };
    validate_domain_ranges(&drone)?;
    Ok(AdvisoryPeerSnapshot { drone, failsafe })
}

fn validate_field_set(state: &CriticalState) -> Result<(), StateError> {
    for field_id in REQUIRED_FIELDS {
        if state.get(field_id).is_none() {
            return Err(StateError::MissingField { field_id });
        }
    }
    for (&field_id, _) in state.iter() {
        if !REQUIRED_FIELDS.contains(&field_id) {
            return Err(StateError::UnknownField { field_id });
        }
    }
    Ok(())
}

fn validate_domain_ranges(drone: &DroneState) -> Result<(), StateError> {
    for (name, value) in [
        ("position.x", drone.position.x),
        ("position.y", drone.position.y),
        ("position.z", drone.position.z),
        ("velocity.vx", drone.velocity.vx),
        ("velocity.vy", drone.velocity.vy),
        ("velocity.vz", drone.velocity.vz),
    ] {
        validate_q16(name, value)?;
    }

    validate_bounded(
        "heading_rad",
        drone.heading_rad,
        -core::f64::consts::TAU,
        core::f64::consts::TAU,
        "-2*pi",
        "2*pi",
    )?;
    validate_bounded(
        "altitude_agl_m",
        drone.altitude_agl_m,
        0.0,
        Q16_16_MAX,
        "0",
        "32767.99998",
    )?;
    validate_bounded(
        "battery_pct",
        f64::from(drone.battery_pct),
        0.0,
        100.0,
        "0",
        "100",
    )?;
    validate_bounded(
        "link_quality",
        f64::from(drone.link_quality),
        0.0,
        1.0,
        "0",
        "1",
    )?;
    Ok(())
}

fn validate_bounded(
    field: &'static str,
    value: f64,
    minimum: f64,
    maximum: f64,
    minimum_label: &'static str,
    maximum_label: &'static str,
) -> Result<(), StateError> {
    if !value.is_finite() {
        return Err(StateError::NonFinite { field });
    }
    if !(minimum..=maximum).contains(&value) {
        return Err(StateError::OutOfRange {
            field,
            minimum: minimum_label,
            maximum: maximum_label,
        });
    }
    validate_q16(field, value)
}

fn validate_q16(field: &'static str, value: f64) -> Result<(), StateError> {
    if !value.is_finite() {
        return Err(StateError::NonFinite { field });
    }
    if !(Q16_16_MIN..=Q16_16_MAX).contains(&value) {
        return Err(StateError::OutOfRange {
            field,
            minimum: "-32768",
            maximum: "32767.99998",
        });
    }
    Ok(())
}

fn set_q16(
    state: &mut CriticalState,
    field_id: u16,
    name: &'static str,
    value: f64,
) -> Result<(), StateError> {
    validate_q16(name, value)?;
    let scaled = (value * Q16_16_SCALE).round();
    if !(i32::MIN as f64..=i32::MAX as f64).contains(&scaled) {
        return Err(StateError::OutOfRange {
            field: name,
            minimum: "-32768",
            maximum: "32767.99998",
        });
    }
    state.set(field_id, SymbolValue::Q16_16(scaled as i32))?;
    Ok(())
}

fn required_u64(state: &CriticalState, field_id: u16) -> Result<u64, StateError> {
    match state.get(field_id) {
        Some(SymbolValue::U64(value)) => Ok(*value),
        Some(_) => Err(StateError::WrongType { field_id }),
        None => Err(StateError::MissingField { field_id }),
    }
}

fn required_q16(state: &CriticalState, field_id: u16) -> Result<f64, StateError> {
    match state.get(field_id) {
        Some(SymbolValue::Q16_16(value)) => Ok(f64::from(*value) / Q16_16_SCALE),
        Some(_) => Err(StateError::WrongType { field_id }),
        None => Err(StateError::MissingField { field_id }),
    }
}

fn failsafe_code(state: &FailSafeState) -> u64 {
    match state {
        FailSafeState::Nominal => 0,
        FailSafeState::AutonomousHold => 1,
        FailSafeState::LowBatteryWarn => 2,
        FailSafeState::ReturnToHome => 3,
        FailSafeState::EmergencyLand => 4,
        FailSafeState::EmergencyDiverge => 5,
        FailSafeState::ControlledDescent => 6,
    }
}

fn failsafe_from_code(code: u64) -> Result<FailSafeState, StateError> {
    Ok(match code {
        0 => FailSafeState::Nominal,
        1 => FailSafeState::AutonomousHold,
        2 => FailSafeState::LowBatteryWarn,
        3 => FailSafeState::ReturnToHome,
        4 => FailSafeState::EmergencyLand,
        5 => FailSafeState::EmergencyDiverge,
        6 => FailSafeState::ControlledDescent,
        _ => return Err(StateError::UnknownFailsafe(code)),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> DroneState {
        DroneState {
            id: NodeId(42),
            position: Position3D {
                x: 123.25,
                y: -45.5,
                z: -20.125,
            },
            velocity: Velocity3D {
                vx: 4.25,
                vy: -1.5,
                vz: 0.125,
            },
            heading_rad: 1.25,
            altitude_agl_m: 20.125,
            battery_pct: 73.5,
            link_quality: 0.875,
            timestamp_ms: 1_725_000_000,
        }
    }

    #[test]
    fn canonical_state_round_trips_with_q16_precision() {
        let original = sample();
        let critical = to_critical_state(&original, &FailSafeState::AutonomousHold).unwrap();
        let decoded = from_critical_state(&critical).unwrap();

        assert_eq!(decoded.drone.id, original.id);
        assert!((decoded.drone.position.x - original.position.x).abs() <= 1.0 / Q16_16_SCALE);
        assert!((decoded.drone.velocity.vy - original.velocity.vy).abs() <= 1.0 / Q16_16_SCALE);
        assert!((decoded.drone.heading_rad - original.heading_rad).abs() <= 1.0 / Q16_16_SCALE);
        assert!((decoded.drone.battery_pct - original.battery_pct).abs() <= 0.000_02);
        assert_eq!(decoded.drone.timestamp_ms, original.timestamp_ms);
        assert_eq!(decoded.failsafe, FailSafeState::AutonomousHold);
        assert_eq!(critical.len(), REQUIRED_FIELDS.len());
    }

    #[test]
    fn non_finite_flight_state_is_rejected() {
        let mut drone = sample();
        drone.position.x = f64::NAN;
        assert!(matches!(
            to_critical_state(&drone, &FailSafeState::Nominal),
            Err(StateError::NonFinite {
                field: "position.x"
            })
        ));

        drone = sample();
        drone.link_quality = f32::INFINITY;
        assert!(matches!(
            to_critical_state(&drone, &FailSafeState::Nominal),
            Err(StateError::NonFinite {
                field: "link_quality"
            })
        ));
    }

    #[test]
    fn out_of_range_and_invalid_operational_values_are_rejected() {
        let mut drone = sample();
        drone.position.x = 32_768.0;
        assert!(matches!(
            to_critical_state(&drone, &FailSafeState::Nominal),
            Err(StateError::OutOfRange {
                field: "position.x",
                ..
            })
        ));

        drone = sample();
        drone.battery_pct = 100.1;
        assert!(matches!(
            to_critical_state(&drone, &FailSafeState::Nominal),
            Err(StateError::OutOfRange {
                field: "battery_pct",
                ..
            })
        ));
    }

    #[test]
    fn malformed_or_extended_critical_schema_is_rejected() {
        let mut critical = to_critical_state(&sample(), &FailSafeState::Nominal).unwrap();
        critical.remove(FIELD_POSITION_NORTH_M);
        assert_eq!(
            from_critical_state(&critical).unwrap_err(),
            StateError::MissingField {
                field_id: FIELD_POSITION_NORTH_M
            }
        );

        let mut critical = to_critical_state(&sample(), &FailSafeState::Nominal).unwrap();
        critical.set(0xffff, SymbolValue::Bool(true)).unwrap();
        assert_eq!(
            from_critical_state(&critical).unwrap_err(),
            StateError::UnknownField { field_id: 0xffff }
        );
    }
}
