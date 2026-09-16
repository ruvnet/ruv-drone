//! Independently implemented, synthetic-world civilian navigation laboratory.
//! No sensor drivers, ROS, autopilot, radio, arming, or actuation interfaces.
use std::cmp::Reverse;
use std::collections::BinaryHeap;
use std::fmt::Write;
use std::time::Instant;

pub const SIZE: [usize; 3] = [32, 24, 12];
const CELLS: usize = SIZE[0] * SIZE[1] * SIZE[2];
pub const RADIUS: f64 = 0.35;
pub const MARGIN: f64 = 0.40;
pub const SPEED: f64 = 3.0;
pub const ACCEL: f64 = 2.0;
pub const DT: f64 = 0.05;
pub const FRESHNESS: f64 = 0.60;
pub const FAULT_TIME: f64 = 3.0;
pub type Vec3 = [f64; 3];

fn add(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn mul(a: Vec3, k: f64) -> Vec3 {
    [a[0] * k, a[1] * k, a[2] * k]
}
pub fn norm(a: Vec3) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}
fn distance(a: Vec3, b: Vec3) -> f64 {
    norm(sub(a, b))
}
fn finite(p: Vec3) -> bool {
    p.iter().all(|x| x.is_finite())
}

#[derive(Clone, Debug)]
pub struct Obstacle {
    pub min: Vec3,
    pub max: Vec3,
    pub unknown: bool,
}
impl Obstacle {
    /// Closed slab intersection, expanded by sphere radius and policy margin.
    /// Tangency is rejected. Works for stationary, parallel, and thin segments.
    fn intersects(&self, a: Vec3, b: Vec3, padding: f64) -> bool {
        let mut lo: f64 = 0.0;
        let mut hi: f64 = 1.0;
        for i in 0..3 {
            let d = b[i] - a[i];
            let low = self.min[i] - padding;
            let high = self.max[i] + padding;
            if d.abs() < 1e-12 {
                if a[i] < low || a[i] > high {
                    return false;
                }
            } else {
                let t1 = (low - a[i]) / d;
                let t2 = (high - a[i]) / d;
                lo = lo.max(t1.min(t2));
                hi = hi.min(t1.max(t2));
                if lo > hi {
                    return false;
                }
            }
        }
        true
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Scenario {
    Inspection,
    Blocked,
    StaleSensor,
    LinkLoss,
    Unknown,
}
impl Scenario {
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "inspection" => Some(Self::Inspection),
            "blocked" => Some(Self::Blocked),
            "stale" => Some(Self::StaleSensor),
            "link" => Some(Self::LinkLoss),
            "unknown" => Some(Self::Unknown),
            _ => None,
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            Self::Inspection => "inspection",
            Self::Blocked => "blocked",
            Self::StaleSensor => "stale",
            Self::LinkLoss => "link",
            Self::Unknown => "unknown",
        }
    }
}

#[derive(Clone)]
pub struct World {
    pub obstacles: Vec<Obstacle>,
    pub start: Vec3,
    pub goal: Vec3,
}
impl World {
    pub fn seeded(seed: u32, scenario: Scenario) -> Self {
        let mut state = u64::from(seed) + 1;
        let mut next = || {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
            (state >> 32) as u32
        };
        let mut obstacles = Vec::new();
        for x in [8.0, 16.0, 24.0] {
            for y in [7.0, 15.0] {
                let height = 3.0 + f64::from(next() % 5);
                let offset = f64::from(next() % 3) * 0.5;
                obstacles.push(Obstacle {
                    min: [x - 2.0, y - 2.0 + offset, 0.0],
                    max: [x + 2.0, y + 2.0 + offset, height],
                    unknown: false,
                });
            }
        }
        // A low wall requires an actual vertical detour, not planar routing.
        obstacles.push(Obstacle {
            min: [13.0, 0.0, 0.0],
            max: [14.0, 24.0, 4.0],
            unknown: false,
        });
        if matches!(scenario, Scenario::Blocked | Scenario::Unknown) {
            obstacles.push(Obstacle {
                min: [15.0, 0.0, 0.0],
                max: [16.0, 24.0, 12.0],
                unknown: scenario == Scenario::Unknown,
            });
        }
        Self {
            obstacles,
            start: [2.0, 3.0, 2.0],
            goal: [29.0, 21.0, 2.0],
        }
    }
    pub fn inside(&self, p: Vec3, pad: f64) -> bool {
        finite(p) && (0..3).all(|i| p[i] >= pad && p[i] <= SIZE[i] as f64 - pad)
    }
    pub fn clear(&self, a: Vec3, b: Vec3, pad: f64) -> bool {
        pad.is_finite()
            && pad >= 0.0
            && self.inside(a, pad)
            && self.inside(b, pad)
            && !self.obstacles.iter().any(|o| o.intersects(a, b, pad))
    }
    pub fn clearance(&self, p: Vec3) -> f64 {
        let mut d = (0..3)
            .map(|i| p[i].min(SIZE[i] as f64 - p[i]))
            .fold(f64::INFINITY, f64::min);
        for o in &self.obstacles {
            let delta = std::array::from_fn(|i| (o.min[i] - p[i]).max(p[i] - o.max[i]).max(0.0));
            d = d.min(norm(delta));
        }
        d - RADIUS
    }
}

fn point(id: usize) -> Vec3 {
    [
        (id % SIZE[0]) as f64,
        ((id / SIZE[0]) % SIZE[1]) as f64,
        (id / (SIZE[0] * SIZE[1])) as f64,
    ]
}
fn index(p: Vec3) -> Option<usize> {
    if !finite(p) || (0..3).any(|i| p[i] < 0.0 || p[i] >= SIZE[i] as f64 || p[i].fract() != 0.0) {
        return None;
    }
    Some(p[0] as usize + SIZE[0] * (p[1] as usize + SIZE[1] * p[2] as usize))
}
fn heuristic(a: Vec3, b: Vec3) -> u32 {
    ((a[0] - b[0]).abs() + (a[1] - b[1]).abs() + (a[2] - b[2]).abs()) as u32
}

#[derive(Debug, PartialEq, Eq)]
pub enum PlanError {
    InvalidEndpoint,
    NoRoute,
    Budget,
}
pub struct Plan {
    pub route: Vec<Vec3>,
    pub expanded: usize,
    pub micros: u128,
}
/// Six-connected deterministic A*, followed by continuously checked shortcuts.
/// Search and memory are bounded by a fixed 9216-cell world. No direct fallback.
pub fn plan(world: &World, budget: usize) -> Result<Plan, PlanError> {
    let clock = Instant::now();
    let start = index(world.start).ok_or(PlanError::InvalidEndpoint)?;
    let goal = index(world.goal).ok_or(PlanError::InvalidEndpoint)?;
    let pad = RADIUS + MARGIN;
    if !world.clear(world.start, world.start, pad) || !world.clear(world.goal, world.goal, pad) {
        return Err(PlanError::InvalidEndpoint);
    }
    let mut g = vec![u32::MAX; CELLS];
    let mut parent = vec![usize::MAX; CELLS];
    let mut visited = vec![false; CELLS];
    let mut queue = BinaryHeap::new();
    g[start] = 0;
    queue.push(Reverse((heuristic(world.start, world.goal), 0, start)));
    let mut expanded = 0;
    while let Some(Reverse((_, cost, id))) = queue.pop() {
        if visited[id] || cost != g[id] {
            continue;
        }
        if expanded >= budget.min(CELLS) {
            return Err(PlanError::Budget);
        }
        visited[id] = true;
        expanded += 1;
        if id == goal {
            let mut raw = vec![point(goal)];
            let mut at = goal;
            while at != start {
                at = parent[at];
                raw.push(point(at));
            }
            raw.reverse();
            let mut route = vec![world.start];
            let mut from = 0;
            while from + 1 < raw.len() {
                let mut to = raw.len() - 1;
                while to > from + 1 && !world.clear(raw[from], raw[to], pad) {
                    to -= 1;
                }
                route.push(raw[to]);
                from = to;
            }
            return Ok(Plan {
                route,
                expanded,
                micros: clock.elapsed().as_micros(),
            });
        }
        let p = point(id);
        for axis in 0..3 {
            for direction in [-1.0, 1.0] {
                let mut q = p;
                q[axis] += direction;
                if let Some(n) = index(q) {
                    if !visited[n] && cost + 1 < g[n] && world.clear(p, q, pad) {
                        g[n] = cost + 1;
                        parent[n] = id;
                        queue.push(Reverse((g[n] + heuristic(q, world.goal), g[n], n)));
                    }
                }
            }
        }
    }
    Err(PlanError::NoRoute)
}

#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub time: f64,
    pub position: Vec3,
    pub velocity: Vec3,
    pub age: f64,
    pub status: &'static str,
    pub clearance: f64,
}
pub struct Run {
    pub seed: u32,
    pub scenario: Scenario,
    pub world: World,
    pub route: Vec<Vec3>,
    pub frames: Vec<Frame>,
    pub outcome: &'static str,
    pub planning_us: u128,
    pub expanded: usize,
}
fn frame(world: &World, time: f64, p: Vec3, v: Vec3, age: f64, status: &'static str) -> Frame {
    Frame {
        time,
        position: p,
        velocity: v,
        age,
        status,
        clearance: world.clearance(p),
    }
}
/// Exact rest-to-rest trapezoidal segment; stops at corners. No rigid-body physics.
fn profile(length: f64, t: f64) -> (f64, f64, f64) {
    let peak = SPEED.min((length * ACCEL).sqrt());
    let rise = peak / ACCEL;
    let cruise = (length / peak - rise).max(0.0);
    let end = 2.0 * rise + cruise;
    let (s, v) = if t <= rise {
        (0.5 * ACCEL * t * t, ACCEL * t)
    } else if t <= rise + cruise {
        (0.5 * peak * rise + peak * (t - rise), peak)
    } else {
        let left = (end - t).max(0.0);
        (length - 0.5 * ACCEL * left * left, ACCEL * left)
    };
    (s.min(length), v, end)
}
pub fn simulate(seed: u32, scenario: Scenario) -> Run {
    let world = World::seeded(seed, scenario);
    let clock = Instant::now();
    let result = plan(&world, CELLS);
    let planning_us = clock.elapsed().as_micros();
    let mut frames = vec![frame(&world, 0.0, world.start, [0.0; 3], 0.0, "READY")];
    let (route, expanded) = match result {
        Ok(p) => (p.route, p.expanded),
        Err(e) => {
            let status = match e {
                PlanError::NoRoute => "HOLD_NO_ROUTE",
                PlanError::Budget => "HOLD_BUDGET",
                PlanError::InvalidEndpoint => "HOLD_INVALID",
            };
            frames[0].status = status;
            return Run {
                seed,
                scenario,
                world,
                route: Vec::new(),
                frames,
                outcome: status,
                planning_us,
                expanded: 0,
            };
        }
    };
    let mut time = 0.0;
    let mut fault = false;
    let mut outcome = "ARRIVED";
    'segments: for pair in route.windows(2) {
        let length = distance(pair[0], pair[1]);
        if length < 1e-12 {
            continue;
        }
        let direction = mul(sub(pair[1], pair[0]), 1.0 / length);
        let end = profile(length, 0.0).2;
        let mut elapsed = 0.0;
        while elapsed < end - 1e-10 {
            let dt = DT.min(end - elapsed);
            let next_time = time + dt;
            let age = if scenario == Scenario::StaleSensor {
                (next_time - FAULT_TIME).max(0.0)
            } else {
                0.0
            };
            if (scenario == Scenario::LinkLoss && next_time >= FAULT_TIME) || age > FRESHNESS {
                fault = true;
                outcome = if scenario == Scenario::LinkLoss {
                    "HOLD_LINK_LOSS"
                } else {
                    "HOLD_STALE_SENSOR"
                };
                break 'segments;
            }
            elapsed += dt;
            time = next_time;
            let (s, v, _) = profile(length, elapsed);
            let p = add(pair[0], mul(direction, s));
            frames.push(frame(&world, time, p, mul(direction, v), age, "FLYING"));
        }
    }
    if fault {
        let last = frames.last().unwrap();
        let mut p = last.position;
        let mut speed = norm(last.velocity);
        let dir = if speed > 0.0 {
            mul(last.velocity, 1.0 / speed)
        } else {
            [0.0; 3]
        };
        // Worst extra detection delay is one simulation tick. Brake on the last
        // validated segment. The rest-to-rest profile guarantees endpoint reserve.
        while speed > 1e-10 {
            let dt = DT.min(speed / ACCEL);
            let v = (speed - ACCEL * dt).max(0.0);
            p = add(p, mul(dir, (speed + v) * 0.5 * dt));
            time += dt;
            let age = if scenario == Scenario::StaleSensor {
                (time - FAULT_TIME).max(0.0)
            } else {
                0.0
            };
            frames.push(frame(&world, time, p, mul(dir, v), age, "BRAKING"));
            speed = v;
        }
    }
    let last = frames.last().unwrap().clone();
    frames.push(frame(
        &world,
        last.time + DT,
        last.position,
        [0.0; 3],
        if scenario == Scenario::StaleSensor {
            (last.time + DT - FAULT_TIME).max(0.0)
        } else {
            0.0
        },
        outcome,
    ));
    Run {
        seed,
        scenario,
        world,
        route,
        frames,
        outcome,
        planning_us,
        expanded,
    }
}

pub struct Validation {
    pub collisions: usize,
    pub geofence: usize,
    pub dynamics: usize,
    pub expected: bool,
}
impl Validation {
    pub fn passed(&self) -> bool {
        self.collisions == 0 && self.geofence == 0 && self.dynamics == 0 && self.expected
    }
}
pub fn validate(run: &Run) -> Validation {
    let mut v = Validation {
        collisions: 0,
        geofence: 0,
        dynamics: 0,
        expected: false,
    };
    for f in &run.frames {
        if !run.world.inside(f.position, RADIUS) {
            v.geofence += 1;
        }
        if !finite(f.velocity) || norm(f.velocity) > SPEED + 1e-8 {
            v.dynamics += 1;
        }
    }
    for pair in run.frames.windows(2) {
        if run
            .world
            .obstacles
            .iter()
            .any(|o| o.intersects(pair[0].position, pair[1].position, RADIUS))
        {
            v.collisions += 1;
        }
        let dt = pair[1].time - pair[0].time;
        if dt <= 0.0 || norm(sub(pair[1].velocity, pair[0].velocity)) > ACCEL * dt + 1e-7 {
            v.dynamics += 1;
        }
    }
    let last = run.frames.last().unwrap();
    v.expected = norm(last.velocity) < 1e-9
        && match run.scenario {
            Scenario::Inspection => {
                run.outcome == "ARRIVED" && distance(last.position, run.world.goal) < 1e-8
            }
            Scenario::Blocked | Scenario::Unknown => {
                run.outcome == "HOLD_NO_ROUTE" && last.position == run.world.start
            }
            Scenario::StaleSensor => run.outcome == "HOLD_STALE_SENSOR",
            Scenario::LinkLoss => run.outcome == "HOLD_LINK_LOSS",
        };
    v
}
fn vec_json(p: Vec3) -> String {
    format!("[{:.6},{:.6},{:.6}]", p[0], p[1], p[2])
}
impl Run {
    pub fn json(&self) -> String {
        let valid = validate(self);
        let mut out = String::with_capacity(self.frames.len() * 160);
        write!(out,"{{\"schema\":1,\"model\":\"kinematic-synthetic-known-map\",\"seed\":{},\"scenario\":\"{}\",\"outcome\":\"{}\",\"planning_us\":{},\"expanded\":{},\"passed\":{},\"collisions\":{},\"geofence_violations\":{},\"dynamics_violations\":{},\"radius\":{},\"margin\":{},\"speed_limit\":{},\"acceleration_limit\":{},\"freshness_limit\":{},\"size\":[32,24,12],\"start\":{},\"goal\":{},\"obstacles\":[",self.seed,self.scenario.name(),self.outcome,self.planning_us,self.expanded,valid.passed(),valid.collisions,valid.geofence,valid.dynamics,RADIUS,MARGIN,SPEED,ACCEL,FRESHNESS,vec_json(self.world.start),vec_json(self.world.goal)).unwrap();
        for (i, o) in self.world.obstacles.iter().enumerate() {
            if i > 0 {
                out.push(',');
            }
            write!(
                out,
                "{{\"min\":{},\"max\":{},\"unknown\":{}}}",
                vec_json(o.min),
                vec_json(o.max),
                o.unknown
            )
            .unwrap();
        }
        out.push_str("],\"route\":[");
        for (i, p) in self.route.iter().enumerate() {
            if i > 0 {
                out.push(',');
            }
            out.push_str(&vec_json(*p));
        }
        out.push_str("],\"frames\":[");
        for (i, f) in self.frames.iter().enumerate() {
            if i > 0 {
                out.push(',');
            }
            write!(out,"{{\"t\":{:.6},\"p\":{},\"v\":{},\"age\":{:.6},\"clearance\":{:.6},\"status\":\"{}\"}}",f.time,vec_json(f.position),vec_json(f.velocity),f.age,f.clearance,f.status).unwrap();
        }
        out.push_str("]}");
        out
    }
}

pub fn evaluate() -> (bool, String) {
    let mut times = Vec::new();
    let mut passed = 0;
    let mut completed = 0;
    let mut collisions = 0;
    let mut geofence = 0;
    let mut dynamics = 0;
    let scenarios = [
        Scenario::Inspection,
        Scenario::Blocked,
        Scenario::StaleSensor,
        Scenario::LinkLoss,
        Scenario::Unknown,
    ];
    let mut rows = String::new();
    for seed in 0..20 {
        for s in scenarios {
            let run = simulate(seed, s);
            let v = validate(&run);
            times.push(run.planning_us);
            if v.passed() {
                passed += 1;
            }
            if run.outcome == "ARRIVED" {
                completed += 1;
            }
            collisions += v.collisions;
            geofence += v.geofence;
            dynamics += v.dynamics;
            if !rows.is_empty() {
                rows.push(',');
            }
            write!(rows,"{{\"seed\":{},\"scenario\":\"{}\",\"outcome\":\"{}\",\"passed\":{},\"planning_us\":{}}}",seed,s.name(),run.outcome,v.passed(),run.planning_us).unwrap();
        }
    }
    times.sort_unstable();
    (passed==100,format!("{{\"runs\":100,\"passed\":{passed},\"completed\":{completed},\"expected_holds\":80,\"collisions\":{collisions},\"geofence_violations\":{geofence},\"dynamics_violations\":{dynamics},\"planning_p50_us\":{},\"planning_p95_us\":{},\"planning_max_us\":{},\"model\":\"synthetic known map; kinematic; no flight readiness claim\",\"results\":[{rows}]}}",times[49],times[94],times[99]))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn hundred_seeded_scenarios_pass() {
        let (ok, report) = evaluate();
        assert!(ok, "{report}");
    }
    #[test]
    fn thin_obstacle_crossing_and_tangency_rejected() {
        let o = Obstacle {
            min: [5.0, 2.0, 1.0],
            max: [5.01, 6.0, 8.0],
            unknown: false,
        };
        assert!(o.intersects([1.0, 4.0, 3.0], [10.0, 4.0, 3.0], 0.0));
        assert!(o.intersects([1.0, 2.0, 3.0], [10.0, 2.0, 3.0], 0.0));
        assert!(!o.intersects([1.0, 0.0, 3.0], [10.0, 0.0, 3.0], 0.5));
    }
    #[test]
    fn blocked_has_no_unsafe_fallback() {
        let w = World::seeded(1, Scenario::Blocked);
        assert!(matches!(plan(&w, CELLS), Err(PlanError::NoRoute)));
    }
    #[test]
    fn budget_exhaustion_is_explicit() {
        assert!(matches!(
            plan(&World::seeded(1, Scenario::Inspection), 0),
            Err(PlanError::Budget)
        ));
    }
    #[test]
    fn nonfinite_and_outside_inputs_rejected() {
        for bad in [
            [f64::NAN, 1.0, 2.0],
            [f64::INFINITY, 1.0, 2.0],
            [-1.0, 1.0, 2.0],
            [32.0, 2.0, 2.0],
            [1.5, 2.0, 2.0],
        ] {
            let mut w = World::seeded(0, Scenario::Inspection);
            w.start = bad;
            assert!(matches!(plan(&w, CELLS), Err(PlanError::InvalidEndpoint)));
        }
    }
    #[test]
    fn replay_is_deterministic_excluding_wall_clock() {
        let a = simulate(42, Scenario::StaleSensor);
        let b = simulate(42, Scenario::StaleSensor);
        assert_eq!(a.route, b.route);
        assert_eq!(a.frames, b.frames);
        assert_eq!(a.outcome, b.outcome);
    }
    #[test]
    fn routes_have_clearance_and_vertical_detour() {
        for seed in 0..20 {
            let w = World::seeded(seed, Scenario::Inspection);
            let p = plan(&w, CELLS).unwrap();
            assert!(p.route.iter().any(|p| p[2] > 4.0));
            assert!(p
                .route
                .windows(2)
                .all(|p| w.clear(p[0], p[1], RADIUS + MARGIN)));
        }
    }
    #[test]
    fn faults_brake_without_teleport_or_acceleration_violation() {
        for s in [Scenario::StaleSensor, Scenario::LinkLoss] {
            let r = simulate(42, s);
            assert!(validate(&r).passed());
            let brake: Vec<_> = r.frames.iter().filter(|f| f.status == "BRAKING").collect();
            assert!(!brake.is_empty());
            assert!(brake
                .windows(2)
                .all(|p| norm(p[1].velocity) <= norm(p[0].velocity) + 1e-9));
            assert!(r
                .frames
                .windows(2)
                .all(|p| distance(p[0].position, p[1].position)
                    <= SPEED * (p[1].time - p[0].time) + 1e-8));
        }
    }
    #[test]
    fn unknown_space_is_not_free_space() {
        let r = simulate(0, Scenario::Unknown);
        assert!(r.world.obstacles.iter().any(|o| o.unknown));
        assert_eq!(r.outcome, "HOLD_NO_ROUTE");
    }
    #[test]
    fn start_equals_goal() {
        let mut w = World::seeded(0, Scenario::Inspection);
        w.goal = w.start;
        let p = plan(&w, CELLS).unwrap();
        assert_eq!(p.route, vec![w.start]);
    }
}
