#![cfg_attr(not(test), no_std)]

use core::{
    future::poll_fn,
    task::{Poll, Waker},
};

use crate::units::{
    Coord, IMillimeters, INum, MillimetersPerSecond, MillimetersPerSecondSquared, UNum,
};
use fixed_sqrt::FastSqrt;

pub mod units;

#[derive(Debug, Clone, Copy, PartialEq)]
enum PlanPhase {
    Accelerating,
    Cruising,
    Decelerating,
    Complete,
}

fn calculate_cruise_velocity(
    current_velocity: MillimetersPerSecond,
    distance: UNum,
    max_accel: MillimetersPerSecondSquared,
) -> MillimetersPerSecond {
    let v0_squared = current_velocity.0 * current_velocity.0;
    let two_a_d = UNum::from_num(2) * max_accel.0 * distance;
    let v_max_squared = v0_squared + two_a_d;

    let v_max = if v_max_squared > UNum::ZERO {
        v_max_squared.fast_sqrt()
    } else {
        UNum::ZERO
    };

    MillimetersPerSecond(v_max)
}

#[derive(Default)]
pub struct PlanBuilder {
    current_position: Option<Coord>,
    current_velocity: Option<MillimetersPerSecond>,
    target_position: Option<Coord>,
    max_accel: Option<MillimetersPerSecondSquared>,
}

impl PlanBuilder {
    pub fn current_position(mut self, v: Option<Coord>) -> Self {
        self.current_position = v;
        self
    }

    pub fn current_velocity(mut self, v: Option<MillimetersPerSecond>) -> Self {
        self.current_velocity = v;
        self
    }

    pub fn target_position(mut self, v: Option<Coord>) -> Self {
        self.target_position = v;
        self
    }

    pub fn max_accel(mut self, v: Option<MillimetersPerSecondSquared>) -> Self {
        self.max_accel = v;
        self
    }

    pub fn build(self) -> Plan {
        let PlanBuilder {
            current_position,
            current_velocity,
            target_position,
            max_accel,
        } = self;
        let current_position = current_position.unwrap();
        let current_velocity = current_velocity.unwrap();
        let target_position = target_position.unwrap();
        let max_accel = max_accel.unwrap();

        let distance_to_target = if target_position.0 .0 >= current_position.0 .0 {
            target_position.0 .0 - current_position.0 .0
        } else {
            current_position.0 .0 - target_position.0 .0
        };

        let cruise_velocity =
            calculate_cruise_velocity(current_velocity, distance_to_target, max_accel);

        let phase = if current_velocity.0 < cruise_velocity.0 {
            PlanPhase::Accelerating
        } else if current_velocity.0 > cruise_velocity.0 {
            PlanPhase::Decelerating
        } else {
            PlanPhase::Cruising
        };

        Plan {
            current_position,
            current_velocity,
            target_position,
            max_accel,
            phase,
            cruise_velocity,
        }
    }
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
    pub fn builder() -> PlanBuilder {
        PlanBuilder::default()
    }

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
            calculate_cruise_velocity(current_velocity, distance_to_target, max_accel);

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
}

#[derive(Debug, PartialEq)]
pub struct MotionSegment {
    pub dist: IMillimeters,
    pub speed: MillimetersPerSecond,
}

impl Iterator for Plan {
    type Item = MotionSegment;

    fn next(&mut self) -> Option<Self::Item> {
        match self.phase {
            PlanPhase::Complete => None,
            PlanPhase::Accelerating => {
                let distance_remaining = if self.target_position.0 .0 >= self.current_position.0 .0
                {
                    self.target_position.0 .0 - self.current_position.0 .0
                } else {
                    self.current_position.0 .0 - self.target_position.0 .0
                };

                if distance_remaining <= UNum::ZERO {
                    self.phase = PlanPhase::Complete;
                    return None;
                }

                let v_current = self.current_velocity.0;
                let v_target = self.cruise_velocity.0;
                let accel = self.max_accel.0;

                let time_to_cruise = (v_target - v_current) / accel;
                let distance_to_accel = v_current * time_to_cruise
                    + UNum::from_num(0.5) * accel * time_to_cruise * time_to_cruise;

                if distance_to_accel >= distance_remaining {
                    let time_to_target =
                        Self::solve_quadratic_for_time(v_current, accel, distance_remaining);
                    let final_velocity = v_current + accel * time_to_target;

                    let direction = if self.target_position.0 .0 >= self.current_position.0 .0 {
                        INum::from_num(1)
                    } else {
                        INum::from_num(-1)
                    };

                    self.current_position.0 .0 = self.target_position.0 .0;
                    self.current_velocity = MillimetersPerSecond(final_velocity);
                    self.phase = PlanPhase::Complete;

                    Some(MotionSegment {
                        dist: IMillimeters(
                            direction * INum::from_num(distance_remaining.to_num::<f32>()),
                        ),
                        speed: MillimetersPerSecond(
                            (v_current + final_velocity) / UNum::from_num(2),
                        ),
                    })
                } else {
                    let direction = if self.target_position.0 .0 >= self.current_position.0 .0 {
                        INum::from_num(1)
                    } else {
                        INum::from_num(-1)
                    };

                    self.current_position.0 .0 =
                        if self.target_position.0 .0 >= self.current_position.0 .0 {
                            self.current_position.0 .0 + distance_to_accel
                        } else {
                            self.current_position.0 .0 - distance_to_accel
                        };
                    self.current_velocity = self.cruise_velocity;
                    self.phase = PlanPhase::Cruising;

                    Some(MotionSegment {
                        dist: IMillimeters(
                            direction * INum::from_num(distance_to_accel.to_num::<f32>()),
                        ),
                        speed: MillimetersPerSecond((v_current + v_target) / UNum::from_num(2)),
                    })
                }
            }
            PlanPhase::Cruising => {
                let distance_remaining = if self.target_position.0 .0 >= self.current_position.0 .0
                {
                    self.target_position.0 .0 - self.current_position.0 .0
                } else {
                    self.current_position.0 .0 - self.target_position.0 .0
                };

                if distance_remaining <= UNum::ZERO {
                    self.phase = PlanPhase::Complete;
                    return None;
                }

                let v_cruise = self.cruise_velocity.0;
                let decel_distance = v_cruise * v_cruise / (UNum::from_num(2) * self.max_accel.0);

                if distance_remaining <= decel_distance {
                    self.phase = PlanPhase::Decelerating;
                    self.next()
                } else {
                    let cruise_distance = distance_remaining - decel_distance;
                    let direction = if self.target_position.0 .0 >= self.current_position.0 .0 {
                        INum::from_num(1)
                    } else {
                        INum::from_num(-1)
                    };

                    self.current_position.0 .0 =
                        if self.target_position.0 .0 >= self.current_position.0 .0 {
                            self.current_position.0 .0 + cruise_distance
                        } else {
                            self.current_position.0 .0 - cruise_distance
                        };

                    Some(MotionSegment {
                        dist: IMillimeters(
                            direction * INum::from_num(cruise_distance.to_num::<f32>()),
                        ),
                        speed: self.cruise_velocity,
                    })
                }
            }
            PlanPhase::Decelerating => {
                let distance_remaining = if self.target_position.0 .0 >= self.current_position.0 .0
                {
                    self.target_position.0 .0 - self.current_position.0 .0
                } else {
                    self.current_position.0 .0 - self.target_position.0 .0
                };

                if distance_remaining <= UNum::ZERO {
                    self.phase = PlanPhase::Complete;
                    return None;
                }

                let v_current = self.current_velocity.0;
                let decel = self.max_accel.0;
                let _time_to_stop = v_current / decel;

                let direction = if self.target_position.0 .0 >= self.current_position.0 .0 {
                    INum::from_num(1)
                } else {
                    INum::from_num(-1)
                };

                self.current_position.0 .0 = self.target_position.0 .0;
                self.current_velocity = MillimetersPerSecond(UNum::ZERO);
                self.phase = PlanPhase::Complete;

                Some(MotionSegment {
                    dist: IMillimeters(
                        direction * INum::from_num(distance_remaining.to_num::<f32>()),
                    ),
                    speed: MillimetersPerSecond(v_current / UNum::from_num(2)),
                })
            }
        }
    }
}

impl Plan {
    fn solve_quadratic_for_time(v0: UNum, a: UNum, d: UNum) -> UNum {
        use fixed_sqrt::FastSqrt;
        let discriminant = v0 * v0 + UNum::from_num(2) * a * d;
        let sqrt_discriminant = discriminant.fast_sqrt();
        let t = (sqrt_discriminant - v0) / a;
        if t > UNum::ZERO {
            t
        } else {
            UNum::ZERO
        }
    }
}

#[derive(Default)]
pub struct StreamingPlanBuilder {
    start_position: Coord,
    max_accel: Option<MillimetersPerSecondSquared>,
}

impl StreamingPlanBuilder {
    pub fn start_position(mut self, v: Coord) -> Self {
        self.start_position = v;
        self
    }

    pub fn max_accel(mut self, v: MillimetersPerSecondSquared) -> Self {
        self.max_accel = Some(v);
        self
    }

    pub fn build(self) -> StreamingPlan {
        let StreamingPlanBuilder {
            start_position,
            max_accel,
        } = self;
        StreamingPlan {
            current_position: start_position,
            current_velocity: MillimetersPerSecond(UNum::ZERO),
            max_accel: max_accel.unwrap(),
            current_plan: None,
            pending_target: None,
            phase: StreamingPhase::Idle,
            waker: None,
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
    waker: Option<Waker>,
}

impl StreamingPlan {
    pub fn builder() -> StreamingPlanBuilder {
        StreamingPlanBuilder::default()
    }

    pub fn new(start_position: Coord, max_accel: MillimetersPerSecondSquared) -> Self {
        Self {
            current_position: start_position,
            current_velocity: MillimetersPerSecond(UNum::ZERO),
            max_accel,
            current_plan: None,
            pending_target: None,
            phase: StreamingPhase::Idle,
            waker: None,
        }
    }

    pub fn add_target(&mut self, target: Coord) -> impl Future<Output = ()> + Unpin + '_ {
        poll_fn(move |cx| match self.phase {
            StreamingPhase::Idle => {
                self.start_move_to(target);
                Poll::Ready(())
            }
            StreamingPhase::ExecutingMove => {
                if self.pending_target.is_some() {
                    self.waker = Some(cx.waker().clone());
                    Poll::Pending
                } else {
                    self.pending_target = Some(target);
                    Poll::Ready(())
                }
            }
            StreamingPhase::WaitingForTarget => {
                self.start_move_to(target);
                Poll::Ready(())
            }
        })
    }

    pub fn finish(&mut self) {
        self.pending_target = None;
        if let Some(w) = self.waker.take() {
            w.wake();
        }
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
        let direction = if segment.dist.0 >= INum::ZERO {
            UNum::from_num(1)
        } else {
            UNum::from_num(-1)
        };

        let distance = UNum::from_num(segment.dist.0.abs().to_num::<f32>());
        self.current_position.0 .0 = if direction >= UNum::from_num(1) {
            self.current_position.0 .0 + distance
        } else {
            self.current_position.0 .0 - distance
        };

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
                }

                if let Some(next_target) = self.pending_target.take() {
                    self.start_move_to(next_target);
                    if let Some(w) = self.waker.take() {
                        w.wake();
                    }
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
    use crate::units::*;

    trait PollUnwrap<T> {
        fn unwrap(self) -> T;
    }

    impl<T> PollUnwrap<T> for Poll<T> {
        #[track_caller]
        fn unwrap(self) -> T {
            match self {
                Poll::Ready(t) => t,
                Poll::Pending => panic!("unwrap() called on Poll::Pending"),
            }
        }
    }

    #[test]
    fn simple_move_forward() {
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
    fn simple_move_backward() {
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
    fn move_with_initial_velocity() {
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
    fn no_move_same_position() {
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
    fn short_move() {
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
    fn trapezoidal_profile() {
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

    mod streaming {
        use core::{pin::Pin, task::Context};

        use super::*;

        #[test]
        fn streaming_single_target() {
            let waker = Waker::noop();
            let mut cx = Context::from_waker(waker);

            let mut streaming_plan = StreamingPlan::new(
                Coord(UMillimeters(UNum::from_num(0.0))),
                MillimetersPerSecondSquared(UNum::from_num(2.0)),
            );

            {
                let mut fut = streaming_plan.add_target(Coord(UMillimeters(UNum::from_num(10.0))));
                Pin::new(&mut fut).poll(&mut cx).unwrap();
            }

            let segments: Vec<MotionSegment> = streaming_plan.collect();
            assert!(!segments.is_empty());

            let total_distance: INum = segments.iter().map(|seg| seg.dist.0).sum();
            assert_eq!(total_distance, INum::from_num(10));
        }

        #[test]
        fn streaming_multiple_targets() {
            let waker = Waker::noop();
            let mut cx = Context::from_waker(waker);

            let mut streaming_plan = StreamingPlan::new(
                Coord(UMillimeters(UNum::from_num(0.0))),
                MillimetersPerSecondSquared(UNum::from_num(2.0)),
            );

            {
                let mut fut = streaming_plan.add_target(Coord(UMillimeters(UNum::from_num(10.0))));
                Pin::new(&mut fut).poll(&mut cx).unwrap();
            }

            let mut segments = Vec::new();
            while let Some(segment) = streaming_plan.next() {
                segments.push(segment);
            }

            {
                let mut fut = streaming_plan.add_target(Coord(UMillimeters(UNum::from_num(20.0))));
                Pin::new(&mut fut).poll(&mut cx).unwrap();
            }

            while let Some(segment) = streaming_plan.next() {
                segments.push(segment);
            }

            assert!(!segments.is_empty());

            let total_distance: INum = segments.iter().map(|seg| seg.dist.0).sum();
            assert_eq!(total_distance, INum::from_num(20));
        }

        #[test]
        fn streaming_buffer_full() {
            let waker = Waker::noop();
            let mut cx = Context::from_waker(waker);

            let mut streaming_plan = StreamingPlan::new(
                Coord(UMillimeters(UNum::from_num(0.0))),
                MillimetersPerSecondSquared(UNum::from_num(2.0)),
            );

            {
                let mut fut = streaming_plan.add_target(Coord(UMillimeters(UNum::from_num(10.0))));
                Pin::new(&mut fut).poll(&mut cx).unwrap();
            }
            {
                let mut fut = streaming_plan.add_target(Coord(UMillimeters(UNum::from_num(20.0))));
                Pin::new(&mut fut).poll(&mut cx).unwrap();
            }

            {
                let mut fut = streaming_plan.add_target(Coord(UMillimeters(UNum::from_num(30.0))));
                let result = Pin::new(&mut fut).poll(&mut cx);
                assert_eq!(result, Poll::Pending);
            }
        }

        #[test]
        fn streaming_idle_state() {
            let mut streaming_plan = StreamingPlan::new(
                Coord(UMillimeters(UNum::from_num(0.0))),
                MillimetersPerSecondSquared(UNum::from_num(2.0)),
            );

            assert_eq!(streaming_plan.next(), None);
        }

        #[test]
        fn streaming_waiting_for_target() {
            let waker = Waker::noop();
            let mut cx = Context::from_waker(waker);

            let mut streaming_plan = StreamingPlan::new(
                Coord(UMillimeters(UNum::from_num(0.0))),
                MillimetersPerSecondSquared(UNum::from_num(2.0)),
            );

            {
                let mut fut = streaming_plan.add_target(Coord(UMillimeters(UNum::from_num(10.0))));
                Pin::new(&mut fut).poll(&mut cx).unwrap();
            }

            let mut segments = Vec::new();
            while let Some(segment) = streaming_plan.next() {
                segments.push(segment);
            }
            assert!(!segments.is_empty());

            assert_eq!(streaming_plan.next(), None);

            {
                let mut fut = streaming_plan.add_target(Coord(UMillimeters(UNum::from_num(20.0))));
                Pin::new(&mut fut).poll(&mut cx).unwrap();
            }
            assert!(streaming_plan.next().is_some());
        }

        #[test]
        fn streaming_finish() {
            let waker = Waker::noop();
            let mut cx = Context::from_waker(waker);

            let mut streaming_plan = StreamingPlan::new(
                Coord(UMillimeters(UNum::from_num(0.0))),
                MillimetersPerSecondSquared(UNum::from_num(2.0)),
            );

            {
                let mut fut = streaming_plan.add_target(Coord(UMillimeters(UNum::from_num(10.0))));
                Pin::new(&mut fut).poll(&mut cx).unwrap();
            }
            {
                let mut fut = streaming_plan.add_target(Coord(UMillimeters(UNum::from_num(20.0))));
                Pin::new(&mut fut).poll(&mut cx).unwrap();
            }

            streaming_plan.finish();

            let segments: Vec<MotionSegment> = streaming_plan.collect();
            assert!(!segments.is_empty());

            let total_distance: INum = segments.iter().map(|seg| seg.dist.0).sum();
            assert_eq!(total_distance, INum::from_num(10.0));
        }
    }
}
