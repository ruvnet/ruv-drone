//! End-to-end: a small fleet transits to a search area with the pulse-and-drift
//! gait, then blooms over a probability hotspot while riding a steady wind,
//! all debited from one shared-style energy budget per drone.

use ruv_jellyfish::field::{GaussianHotspot, HotspotField, UniformFlow};
use ruv_jellyfish::{EnergyModel, Gait, JellyfishController, Vec3};

#[test]
fn cruise_then_bloom_within_budget() {
    let budget = 800_000.0_f64; // J/kg
    let energy = EnergyModel::default();

    // Four drones spread around a 400 m box, hotspot near one corner.
    let hotspot = Vec3::new(90.0, 90.0, 0.0);
    let value = HotspotField::new(vec![GaussianHotspot { centre: hotspot, peak: 1.0, sigma: 45.0 }]);
    let wind = UniformFlow(Vec3::new(1.2, 0.8, 0.0));

    let mut positions = vec![
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(400.0, 0.0, 0.0),
        Vec3::new(0.0, 400.0, 0.0),
        Vec3::new(400.0, 400.0, 0.0),
    ];
    let mut drones: Vec<JellyfishController> = (0..positions.len())
        .map(|_| JellyfishController::with_budget(budget))
        .collect();

    // --- Phase 1: cruise (transit gait) for 60 s ---
    for _ in 0..1200 {
        for d in &mut drones {
            d.cruise(0.05);
        }
    }
    for d in &drones {
        let t = d.gait_telemetry();
        assert!(t.distance > 0.0, "cruise should cover ground");
        // Pulse-drift must beat the constant-thrust actuation reference.
        assert!(
            t.actuation_energy_per_metre() < energy.actuation_energy_per_metre(Gait::ConstantThrust),
            "gait not saving energy vs constant thrust"
        );
    }

    // --- Phase 2: bloom over the hotspot for 90 s ---
    let spread_before = mean_dist(&positions, hotspot);
    for _ in 0..900 {
        let snapshot = positions.clone();
        for i in 0..positions.len() {
            let neighbours: Vec<Vec3> = snapshot
                .iter()
                .enumerate()
                .filter(|(j, _)| *j != i)
                .map(|(_, &p)| p)
                .collect();
            let step = drones[i].loiter(positions[i], &neighbours, &value, &wind, 0.0, 0.1);
            positions[i] = positions[i].add(step.command.expected_ground_velocity.scale(0.1));
        }
    }
    let spread_after = mean_dist(&positions, hotspot);

    // The smack should have contracted toward the hotspot...
    assert!(
        spread_after < spread_before,
        "fleet did not aggregate: {spread_before} -> {spread_after}"
    );
    // ...without any drone exhausting its budget or collapsing onto another.
    for d in &drones {
        assert!(!d.depleted(), "drone ran out of energy inside the mission budget");
        assert!(d.budget_remaining() > 0.0);
    }
    for i in 0..positions.len() {
        for j in (i + 1)..positions.len() {
            assert!(
                positions[i].distance_to(positions[j]) > 1.0,
                "drones collapsed together"
            );
        }
    }
}

fn mean_dist(pos: &[Vec3], c: Vec3) -> f64 {
    pos.iter().map(|p| p.distance_to(c)).sum::<f64>() / pos.len() as f64
}
