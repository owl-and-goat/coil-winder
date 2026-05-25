#![cfg_attr(not(test), no_std)]

use units::{Coord, IMillimeters, INum, MillimetersPerSecond, MillimetersPerSecondSquared, UNum};

#[derive(Debug, Clone, Copy, PartialEq)]
enum PlanPhase {
    Accelerating,
    Cruising,
    Decelerating,
    Complete,
}

pub struct Plan {
    current_position: Coord,
    current_velocity: MillimetersPerSecond,
    target_position: Coord,
    max_accel: MillimetersPerSecondSquared,
    phase: PlanPhase,
    cruise_velocity: MillimetersPerSecond,
}

impl Plan {
    pub fn new(
        current_position: Coord,
        current_velocity: MillimetersPerSecond,
        target_position: Coord,
        max_accel: MillimetersPerSecondSquared,
    ) -> Self {
        let distance_to_target = if target_position.0 .0 >= current_position.0 .0 {
            target_position.0 .0 - current_position.0 .0
        } else {
            current_position.0 .0 - target_position.0 .0
        };

        let cruise_velocity =
            Self::calculate_cruise_velocity(current_velocity, distance_to_target, max_accel);

        let phase = if current_velocity.0 < cruise_velocity.0 {
            PlanPhase::Accelerating
        } else if current_velocity.0 > cruise_velocity.0 {
            PlanPhase::Decelerating
        } else {
            PlanPhase::Cruising
        };

        Self {
            current_position,
            current_velocity,
            target_position,
            max_accel,
            phase,
            cruise_velocity,
        }
    }

    fn calculate_cruise_velocity(
        current_velocity: MillimetersPerSecond,
        distance: UNum,
        max_accel: MillimetersPerSecondSquared,
    ) -> MillimetersPerSecond {
        // For a move that accelerates from v0 to v_cruise then decelerates to 0,
        // with d_accel + d_decel = distance:
        //   (v_cruise² - v0²)/(2a) + v_cruise²/(2a) = distance
        //   v_cruise² = a·distance + v0²/2
        let v0_half_squared =
            current_velocity.0.saturating_mul(current_velocity.0) / UNum::from_num(2);
        let a_d = max_accel.0.saturating_mul(distance);
        let v_max_squared = v0_half_squared.saturating_add(a_d);

        use fixed_sqrt::FastSqrt;
        let v_max = if v_max_squared > UNum::ZERO {
            v_max_squared.fast_sqrt()
        } else {
            UNum::ZERO
        };

        MillimetersPerSecond(v_max)
    }
}

#[derive(Debug, PartialEq)]
pub struct MotionSegment {
    pub dist: IMillimeters,
    pub speed: MillimetersPerSecond,
}

impl Iterator for Plan {
    type Item = MotionSegment;

    fn next(&mut self) -> Option<Self::Item> {
        // INum can only represent distances up to just under 2^21. For larger
        // distances we emit chunks and stay in the current phase until done.
        const MAX_CHUNK: UNum = UNum::from_bits(i32::MAX as u32);

        let forward = self.target_position.0 .0 >= self.current_position.0 .0;
        let dr = if forward {
            self.target_position.0 .0 - self.current_position.0 .0
        } else {
            self.current_position.0 .0 - self.target_position.0 .0
        };

        // Convert a non-negative UNum distance (≤ MAX_CHUNK) to a signed INum
        // displacement, applying direction.
        let make_dist = |d: UNum| -> IMillimeters {
            let bits = d.to_bits() as i32; // safe: d <= MAX_CHUNK <= i32::MAX
            IMillimeters(if forward {
                INum::from_bits(bits)
            } else {
                -INum::from_bits(bits)
            })
        };

        // Advance current_position by `chunk` in the correct direction.
        let advance = |pos: UNum, chunk: UNum| -> UNum {
            if forward {
                pos + chunk
            } else {
                pos - chunk
            }
        };

        match self.phase {
            PlanPhase::Complete => None,
            PlanPhase::Accelerating => {
                if dr == UNum::ZERO {
                    self.phase = PlanPhase::Complete;
                    return None;
                }

                let v_current = self.current_velocity.0;
                let v_target = self.cruise_velocity.0;
                let accel = self.max_accel.0;

                let time_to_cruise = if v_target > v_current {
                    (v_target - v_current)
                        .checked_div(accel)
                        .unwrap_or(UNum::MAX)
                } else {
                    UNum::ZERO
                };
                let distance_to_accel = v_current.saturating_mul(time_to_cruise).saturating_add(
                    UNum::from_num(0.5_f32)
                        .saturating_mul(accel)
                        .saturating_mul(time_to_cruise)
                        .saturating_mul(time_to_cruise),
                );

                if distance_to_accel >= dr {
                    // Won't reach cruise velocity; head straight to target.
                    let chunk = dr.min(MAX_CHUNK);
                    let t = Self::solve_quadratic_for_time(v_current, accel, chunk);
                    let v_end = v_current.saturating_add(accel.saturating_mul(t));

                    self.current_position.0 .0 = advance(self.current_position.0 .0, chunk);
                    self.current_velocity = MillimetersPerSecond(v_end);
                    if dr <= MAX_CHUNK {
                        self.phase = PlanPhase::Complete;
                    }

                    Some(MotionSegment {
                        dist: make_dist(chunk),
                        speed: MillimetersPerSecond(
                            v_current.saturating_add(v_end) / UNum::from_num(2),
                        ),
                    })
                } else {
                    // Will reach cruise velocity.
                    let chunk = distance_to_accel.min(MAX_CHUNK);
                    self.current_position.0 .0 = advance(self.current_position.0 .0, chunk);

                    if distance_to_accel <= MAX_CHUNK {
                        // Full acceleration phase done.
                        self.current_velocity = self.cruise_velocity;
                        self.phase = PlanPhase::Cruising;
                        Some(MotionSegment {
                            dist: make_dist(chunk),
                            speed: MillimetersPerSecond(
                                v_current.saturating_add(v_target) / UNum::from_num(2),
                            ),
                        })
                    } else {
                        // Partial chunk: compute velocity reached at end of chunk.
                        let v_end = Self::velocity_after_accel(v_current, accel, chunk);
                        self.current_velocity = MillimetersPerSecond(v_end);
                        Some(MotionSegment {
                            dist: make_dist(chunk),
                            speed: MillimetersPerSecond(
                                v_current.saturating_add(v_end) / UNum::from_num(2),
                            ),
                        })
                    }
                }
            }
            PlanPhase::Cruising => {
                if dr == UNum::ZERO {
                    self.phase = PlanPhase::Complete;
                    return None;
                }

                let v_cruise = self.cruise_velocity.0;
                let decel_distance = v_cruise
                    .saturating_mul(v_cruise)
                    .checked_div(UNum::from_num(2_u32).saturating_mul(self.max_accel.0))
                    .unwrap_or(UNum::MAX);

                if dr <= decel_distance {
                    self.phase = PlanPhase::Decelerating;
                    self.next()
                } else {
                    let cruise_distance = dr - decel_distance;
                    let chunk = cruise_distance.min(MAX_CHUNK);
                    self.current_position.0 .0 = advance(self.current_position.0 .0, chunk);
                    Some(MotionSegment {
                        dist: make_dist(chunk),
                        speed: self.cruise_velocity,
                    })
                }
            }
            PlanPhase::Decelerating => {
                if dr == UNum::ZERO {
                    self.phase = PlanPhase::Complete;
                    return None;
                }

                let v_current = self.current_velocity.0;
                let chunk = dr.min(MAX_CHUNK);
                self.current_position.0 .0 = advance(self.current_position.0 .0, chunk);
                if dr <= MAX_CHUNK {
                    self.current_velocity = MillimetersPerSecond(UNum::ZERO);
                    self.phase = PlanPhase::Complete;
                }

                Some(MotionSegment {
                    dist: make_dist(chunk),
                    speed: MillimetersPerSecond(v_current / UNum::from_num(2)),
                })
            }
        }
    }
}

impl Plan {
    fn solve_quadratic_for_time(v0: UNum, a: UNum, d: UNum) -> UNum {
        use fixed_sqrt::FastSqrt;
        let discriminant = v0
            .saturating_mul(v0)
            .saturating_add(UNum::from_num(2_u32).saturating_mul(a).saturating_mul(d));
        let sqrt_discriminant = discriminant.fast_sqrt();
        if sqrt_discriminant > v0 {
            (sqrt_discriminant - v0).checked_div(a).unwrap_or(UNum::MAX)
        } else {
            UNum::ZERO
        }
    }

    fn velocity_after_accel(v0: UNum, accel: UNum, dist: UNum) -> UNum {
        use fixed_sqrt::FastSqrt;
        let v_sq = v0.saturating_mul(v0).saturating_add(
            UNum::from_num(2_u32)
                .saturating_mul(accel)
                .saturating_mul(dist),
        );
        if v_sq > UNum::ZERO {
            v_sq.fast_sqrt()
        } else {
            UNum::ZERO
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum StreamingError {
    BufferFull,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum StreamingPhase {
    Idle,
    ExecutingMove,
    WaitingForTarget,
}

pub struct StreamingPlan {
    current_position: Coord,
    current_velocity: MillimetersPerSecond,
    max_accel: MillimetersPerSecondSquared,
    current_plan: Option<Plan>,
    pending_target: Option<Coord>,
    phase: StreamingPhase,
}

impl StreamingPlan {
    pub fn new(start_position: Coord, max_accel: MillimetersPerSecondSquared) -> Self {
        assert!(!max_accel.0.is_zero(), "max_accel must be nonzero");

        Self {
            current_position: start_position,
            current_velocity: MillimetersPerSecond(UNum::ZERO),
            max_accel,
            current_plan: None,
            pending_target: None,
            phase: StreamingPhase::Idle,
        }
    }

    pub fn add_target(&mut self, target: Coord) -> Result<(), StreamingError> {
        match self.phase {
            StreamingPhase::Idle => {
                self.start_move_to(target);
                Ok(())
            }
            StreamingPhase::ExecutingMove => {
                if self.pending_target.is_some() {
                    Err(StreamingError::BufferFull)
                } else {
                    self.pending_target = Some(target);
                    Ok(())
                }
            }
            StreamingPhase::WaitingForTarget => {
                self.start_move_to(target);
                Ok(())
            }
        }
    }

    pub fn finish(&mut self) {
        self.pending_target = None;
    }

    fn start_move_to(&mut self, target: Coord) {
        self.current_plan = Some(Plan::new(
            self.current_position,
            self.current_velocity,
            target,
            self.max_accel,
        ));
        self.phase = StreamingPhase::ExecutingMove;
    }

    fn update_state_from_segment(&mut self, segment: &MotionSegment) {
        self.current_position.0 .0 = self.current_position.0 .0.add_signed(segment.dist.0);
        self.current_velocity = segment.speed;
    }
}

impl Iterator for StreamingPlan {
    type Item = MotionSegment;

    fn next(&mut self) -> Option<Self::Item> {
        match self.phase {
            StreamingPhase::Idle => None,
            StreamingPhase::ExecutingMove => {
                if let Some(ref mut plan) = self.current_plan {
                    if let Some(segment) = plan.next() {
                        self.update_state_from_segment(&segment);
                        return Some(segment);
                    }
                    // Plan exhausted: it always decelerates to rest, so reset velocity.
                    self.current_velocity = MillimetersPerSecond(UNum::ZERO);
                }

                if let Some(next_target) = self.pending_target.take() {
                    self.start_move_to(next_target);
                    self.next()
                } else {
                    self.phase = StreamingPhase::WaitingForTarget;
                    None
                }
            }
            StreamingPhase::WaitingForTarget => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fixed::types::extra::U20;
    use fixed::{FixedI64, FixedU32};
    use units::UMillimeters;
    // Wide fixed-point type for intermediate products that would overflow INum.
    type Wide = FixedI64<U20>;
    use hegel::generators as gs;
    use hegel::TestCase;

    #[test]
    fn test_simple_move_forward() {
        let plan = Plan::new(
            Coord(UMillimeters(UNum::from_num(0.0))),
            MillimetersPerSecond(UNum::from_num(0.0)),
            Coord(UMillimeters(UNum::from_num(10.0))),
            MillimetersPerSecondSquared(UNum::from_num(2.0)),
        );

        let segments: Vec<MotionSegment> = plan.collect();
        assert!(!segments.is_empty());

        let total_distance: INum = segments.iter().map(|seg| seg.dist.0).sum();

        let expected = INum::from_num(10.0);
        assert!((total_distance - expected).abs() < INum::from_num(0.1));
    }

    #[test]
    fn test_simple_move_backward() {
        let plan = Plan::new(
            Coord(UMillimeters(UNum::from_num(10.0))),
            MillimetersPerSecond(UNum::from_num(0.0)),
            Coord(UMillimeters(UNum::from_num(0.0))),
            MillimetersPerSecondSquared(UNum::from_num(2.0)),
        );

        let segments: Vec<MotionSegment> = plan.collect();
        assert!(!segments.is_empty());

        let total_distance: INum = segments.iter().map(|seg| seg.dist.0).sum();

        let expected = INum::from_num(-10.0);
        assert!((total_distance - expected).abs() < INum::from_num(0.1));
    }

    #[test]
    fn test_move_with_initial_velocity() {
        let plan = Plan::new(
            Coord(UMillimeters(UNum::from_num(0.0))),
            MillimetersPerSecond(UNum::from_num(5.0)),
            Coord(UMillimeters(UNum::from_num(20.0))),
            MillimetersPerSecondSquared(UNum::from_num(2.0)),
        );

        let segments: Vec<MotionSegment> = plan.collect();
        assert!(!segments.is_empty());

        let total_distance: INum = segments.iter().map(|seg| seg.dist.0).sum();

        let expected = INum::from_num(20.0);
        assert!((total_distance - expected).abs() < INum::from_num(0.1));
    }

    #[test]
    fn test_no_move_same_position() {
        let plan = Plan::new(
            Coord(UMillimeters(UNum::from_num(5.0))),
            MillimetersPerSecond(UNum::from_num(0.0)),
            Coord(UMillimeters(UNum::from_num(5.0))),
            MillimetersPerSecondSquared(UNum::from_num(2.0)),
        );

        let segments: Vec<MotionSegment> = plan.collect();
        assert!(segments.is_empty());
    }

    #[test]
    fn test_short_move() {
        let plan = Plan::new(
            Coord(UMillimeters(UNum::from_num(0.0))),
            MillimetersPerSecond(UNum::from_num(0.0)),
            Coord(UMillimeters(UNum::from_num(1.0))),
            MillimetersPerSecondSquared(UNum::from_num(10.0)),
        );

        let segments: Vec<MotionSegment> = plan.collect();
        assert!(!segments.is_empty());

        let total_distance: INum = segments.iter().map(|seg| seg.dist.0).sum();

        assert!((total_distance - INum::from_num(1.0)).abs() < INum::from_num(0.1));
        assert!(segments.iter().all(|seg| seg.speed.0 > UNum::ZERO));
    }

    #[test]
    fn test_trapezoidal_profile() {
        let plan = Plan::new(
            Coord(UMillimeters(UNum::from_num(0.0))),
            MillimetersPerSecond(UNum::from_num(0.0)),
            Coord(UMillimeters(UNum::from_num(100.0))),
            MillimetersPerSecondSquared(UNum::from_num(1.0)),
        );

        let segments: Vec<MotionSegment> = plan.collect();
        assert!(!segments.is_empty());

        let total_distance: INum = segments.iter().map(|seg| seg.dist.0).sum();

        let expected = INum::from_num(100.0);
        assert!((total_distance - expected).abs() < INum::from_num(0.1));

        assert!(segments.iter().all(|seg| seg.speed.0 > UNum::ZERO));
        assert!(segments.iter().all(|seg| seg.dist.0.abs() > INum::ZERO));
    }

    #[test]
    fn test_streaming_single_target() {
        let mut streaming_plan = StreamingPlan::new(
            Coord(UMillimeters(UNum::from_num(0.0))),
            MillimetersPerSecondSquared(UNum::from_num(2.0)),
        );

        streaming_plan
            .add_target(Coord(UMillimeters(UNum::from_num(10.0))))
            .unwrap();

        let segments: Vec<MotionSegment> = streaming_plan.collect();
        assert!(!segments.is_empty());

        let total_distance: INum = segments.iter().map(|seg| seg.dist.0).sum();
        assert!((total_distance - INum::from_num(10.0)).abs() < INum::from_num(0.1));
    }

    #[test]
    fn test_streaming_multiple_targets() {
        let mut streaming_plan = StreamingPlan::new(
            Coord(UMillimeters(UNum::from_num(0.0))),
            MillimetersPerSecondSquared(UNum::from_num(2.0)),
        );

        streaming_plan
            .add_target(Coord(UMillimeters(UNum::from_num(10.0))))
            .unwrap();

        let mut segments = Vec::new();
        while let Some(segment) = streaming_plan.next() {
            segments.push(segment);
        }

        streaming_plan
            .add_target(Coord(UMillimeters(UNum::from_num(20.0))))
            .unwrap();

        while let Some(segment) = streaming_plan.next() {
            segments.push(segment);
        }

        assert!(!segments.is_empty());

        let total_distance: INum = segments.iter().map(|seg| seg.dist.0).sum();
        assert!((total_distance - INum::from_num(20.0)).abs() < INum::from_num(0.1));
    }

    #[test]
    fn test_streaming_buffer_full() {
        let mut streaming_plan = StreamingPlan::new(
            Coord(UMillimeters(UNum::from_num(0.0))),
            MillimetersPerSecondSquared(UNum::from_num(2.0)),
        );

        streaming_plan
            .add_target(Coord(UMillimeters(UNum::from_num(10.0))))
            .unwrap();
        streaming_plan
            .add_target(Coord(UMillimeters(UNum::from_num(20.0))))
            .unwrap();

        let result = streaming_plan.add_target(Coord(UMillimeters(UNum::from_num(30.0))));
        assert_eq!(result, Err(StreamingError::BufferFull));
    }

    #[test]
    fn test_streaming_idle_state() {
        let mut streaming_plan = StreamingPlan::new(
            Coord(UMillimeters(UNum::from_num(0.0))),
            MillimetersPerSecondSquared(UNum::from_num(2.0)),
        );

        assert_eq!(streaming_plan.next(), None);
    }

    #[test]
    fn test_streaming_waiting_for_target() {
        let mut streaming_plan = StreamingPlan::new(
            Coord(UMillimeters(UNum::from_num(0.0))),
            MillimetersPerSecondSquared(UNum::from_num(2.0)),
        );

        streaming_plan
            .add_target(Coord(UMillimeters(UNum::from_num(10.0))))
            .unwrap();

        let mut segments = Vec::new();
        while let Some(segment) = streaming_plan.next() {
            segments.push(segment);
        }
        assert!(!segments.is_empty());

        assert_eq!(streaming_plan.next(), None);

        streaming_plan
            .add_target(Coord(UMillimeters(UNum::from_num(20.0))))
            .unwrap();
        assert!(streaming_plan.next().is_some());
    }

    #[test]
    fn test_streaming_finish() {
        let mut streaming_plan = StreamingPlan::new(
            Coord(UMillimeters(UNum::from_num(0.0))),
            MillimetersPerSecondSquared(UNum::from_num(2.0)),
        );

        streaming_plan
            .add_target(Coord(UMillimeters(UNum::from_num(10.0))))
            .unwrap();
        streaming_plan
            .add_target(Coord(UMillimeters(UNum::from_num(20.0))))
            .unwrap();

        streaming_plan.finish();

        let segments: Vec<MotionSegment> = streaming_plan.collect();
        assert!(!segments.is_empty());

        let total_distance: INum = segments.iter().map(|seg| seg.dist.0).sum();
        assert!((total_distance - INum::from_num(10.0)).abs() < INum::from_num(0.1));
    }

    // Verify that each segment's implied acceleration doesn't exceed max_accel.
    //
    // Each segment is a constant-velocity move: the motor runs at `speed` for
    // `dist` steps. Acceleration comes from the speed change between segments.
    //
    // The Plan encodes the intended entry/exit velocities by setting
    //   speed = (v_entry + v_exit) / 2
    // for each segment, so we can recover v_exit = 2·speed - v_entry.
    // Tracking v across segments then gives us the implied velocity profile,
    // and we can check (v_exit - v_entry)·speed / dist ≤ max_accel.
    //
    // Distance is bounded to ≥1000 bits (≈1mm) so that fixed-point quantisation
    // errors are well under the 10% tolerance. Accel is bounded to 1–100 mm/s².
    #[hegel::test]
    fn max_accel_respected(tc: TestCase) {
        let target_bits = tc.draw(gs::integers::<u32>().min_value(1000).max_value(1000 * 1024));
        let accel_bits = tc.draw(gs::integers::<u32>().min_value(1024).max_value(100 * 1024));

        let start = Coord(UMillimeters(UNum::ZERO));
        let target = Coord(UMillimeters(UNum::from_bits(target_bits)));
        let max_accel = MillimetersPerSecondSquared(UNum::from_bits(accel_bits));

        let mut v = INum::ZERO;

        for seg in Plan::new(start, MillimetersPerSecond(UNum::ZERO), target, max_accel) {
            let speed = seg.speed.0;
            let dist = seg.dist.0.abs();
            let v_end = INum::from_num(speed) * INum::from_num(2_u32) - v; // invert speed = (v_entry + v_exit)/2
            if dist > INum::from_bits(1) {
                // Intermediate product (mm/s)² needs Wide to avoid INum overflow.
                let accel = (Wide::from_num(v_end - v) * Wide::from_num(speed)
                    / Wide::from_num(dist))
                .abs();
                assert!(
                    accel * Wide::from_num(10_u32)
                        <= Wide::from_num(max_accel.0) * Wide::from_num(11_u32),
                    "acceleration {accel} exceeds max_accel {}",
                    max_accel.0
                );
            }
            v = v_end;
        }
    }

    // Same acceleration check as max_accel_respected, but run through StreamingPlan
    // across a sequence of targets with random direction (forward or backward).
    //
    // Each move has a magnitude ≥1000 bits (≈1mm) for good fixed-point precision
    // and a random direction drawn independently.  Positions are accumulated from
    // a mid-range start and clamped to the valid UNum range.
    //
    // Velocity is tracked as a speed magnitude using the same v_exit = 2·speed - v
    // encoding described in max_accel_respected.  This works for both directions
    // because `speed` is always the unsigned average of the entry and exit speeds.
    #[hegel::test]
    fn streaming_max_accel_respected(tc: TestCase) {
        let accel_bits = tc.draw(gs::integers::<u32>().min_value(1024).max_value(100 * 1024));
        let start_bits = tc.draw(gs::integers::<u32>().min_value(50_000).max_value(500_000));
        let magnitudes = tc
            .draw(gs::vecs(gs::integers::<u32>().min_value(1000).max_value(5_000_000)).min_size(2));
        // true = forward, false = backward
        let directions = tc.draw(gs::vecs(gs::booleans()).min_size(2));

        let max_accel = MillimetersPerSecondSquared(UNum::from_bits(accel_bits));
        let mut pos = start_bits as i64;
        let mut targets = magnitudes.into_iter().zip(directions).map(|(mag, dir)| {
            let offset = if dir { mag as i64 } else { -(mag as i64) };
            pos = (pos + offset).clamp(0, u32::MAX as i64);
            Coord(UMillimeters(UNum::from_bits(pos as u32)))
        });

        let start = Coord(UMillimeters(UNum::from_bits(start_bits)));
        let mut plan = StreamingPlan::new(start, max_accel);

        plan.add_target(targets.next().unwrap()).unwrap();

        let mut v = INum::ZERO;
        for target in targets {
            plan.add_target(target).unwrap();
            for seg in &mut plan {
                let speed = seg.speed.0;
                let dist = seg.dist.0.abs();
                let v_end = INum::from_num(speed) * INum::from_num(2_u32) - v; // invert speed = (v_entry + v_exit)/2
                if dist > INum::from_bits(1) {
                    let accel = (Wide::from_num(v_end - v) * Wide::from_num(speed)
                        / Wide::from_num(dist))
                    .abs();
                    assert!(
                        accel * Wide::from_num(10_u32)
                            <= Wide::from_num(max_accel.0) * Wide::from_num(11_u32),
                        "acceleration {accel} exceeds max_accel {}",
                        max_accel.0
                    );
                }
                v = v_end;
            }
        }
    }

    // Verify that when all targets have been consumed and the StreamingPlan drains to
    // completion, the implied exit velocity of the final segment is ≈ 0.
    //
    // Using the same v_exit = 2·speed - v encoding as max_accel_respected: a final
    // segment with speed = v_entry/2 implies v_exit = 0, i.e. the Plan encodes a
    // proper deceleration to rest rather than cutting off at speed.
    #[hegel::test]
    fn streaming_decelerates_to_stop(tc: TestCase) {
        let accel_bits = tc.draw(gs::integers::<u32>().min_value(1024).max_value(100 * 1024));
        let target_bits = tc.draw(gs::integers::<u32>().min_value(1000).max_value(1000 * 1024));

        let start = Coord(UMillimeters(UNum::ZERO));
        let target = Coord(UMillimeters(UNum::from_bits(target_bits)));
        let max_accel = MillimetersPerSecondSquared(UNum::from_bits(accel_bits));

        let mut plan = StreamingPlan::new(start, max_accel);
        plan.add_target(target).unwrap();

        let mut v = INum::ZERO;
        for seg in &mut plan {
            v = INum::from_num(seg.speed.0) * INum::from_num(2_u32) - v; // invert speed = (v_entry + v_exit)/2
        }

        // Division-by-2 in the decel segment truncates, introducing at most 2 ULPs of error.
        assert!(
            v.abs() <= INum::from_bits(2),
            "final velocity should be ~0, got {v} (target_bits={target_bits}, accel_bits={accel_bits})"
        );
    }

    #[hegel::test]
    fn sum_of_distances_matches(tc: TestCase) {
        let start_position = tc.draw(gs::integers());
        let start_position = Coord(UMillimeters::from(FixedU32::from_bits(start_position)));

        let max_accel = tc.draw(gs::integers().min_value(1));
        let max_accel = MillimetersPerSecondSquared(FixedU32::from_bits(max_accel));

        let mut plan = StreamingPlan::new(start_position, max_accel);

        let targets = tc.draw(gs::vecs(gs::integers::<u32>()).min_size(2));
        let mut targets = targets
            .into_iter()
            .map(|i| Coord(UMillimeters(FixedU32::from_bits(i))));
        plan.add_target(targets.next().unwrap()).unwrap();

        let mut cur_position = start_position;

        let mut last_target = None;
        for target in targets {
            plan.add_target(target).unwrap();
            for step in &mut plan {
                cur_position.0 .0 = cur_position.0 .0.add_signed(step.dist.0);
            }
            last_target = Some(target);
        }

        assert_eq!(cur_position, last_target.unwrap());
    }
}
