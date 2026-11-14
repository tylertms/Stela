//! Admission and completion policy shared by the distinct execution perimeters.
//! These methods neither call providers nor grant CPU, IRQ, current or root access.
use super::*;
/// The number of [`Lane`]s.
#[cfg(target_arch = "x86_64")]
pub(super) const LANES: usize = 7;
/// A dense index for each [`Lane`].
#[cfg(target_arch = "x86_64")]
pub(super) const fn lane_index(lane: Lane) -> usize {
    match lane {
        Lane::Foreground => 0,
        Lane::Notify => 1,
        Lane::General => 2,
        Lane::Service => 3,
        Lane::Firmware => 4,
        Lane::Portal => 5,
        Lane::Gpe => 6,
    }
}
impl<const N: usize> State<N> {
    pub(super) fn new() -> Self {
        Self {
            #[cfg(all(target_arch = "x86_64", feature = "linux-kernel"))]
            inactive: [const { None }; MAX_CONTINUATIONS],
            jobs: [None; N],
            reservations: [0; N],
            #[cfg(target_arch = "x86_64")]
            next_reservation: 1,
            layout: Layout::Ordinary,
            stack_profile: None,
            stack_catalog: stack_requirement::Catalog::empty(),
            next: 1,
            closed: false,
            poisoned: false,
            #[cfg(target_arch = "x86_64")]
            scheduling: scheduling::Board::new(),
            #[cfg(target_arch = "x86_64")]
            resources: credits::Resources::new(),
        }
    }
    pub(super) fn enqueue(
        &mut self,
        lane: Lane,
        callback: Callback,
        context: *mut c_void,
        requested: Option<StackRequirement>,
        abandoned: bool,
    ) -> Result<u64, WorkError> {
        if self.closed || self.poisoned || abandoned {
            return Err(WorkError::Closed);
        }
        let requirement = match (self.stack_profile, requested) {
            (None, Some(_)) => return Err(WorkError::Unavailable),
            (None, None) => None,
            (Some(profile), requested) => {
                let requirement = requested.unwrap_or_else(|| profile.default_for(lane));
                if !self.stack_catalog.supports(lane, requirement) {
                    return Err(WorkError::Unavailable);
                }
                Some(requirement)
            }
        };
        let ranges = self.layout.admission(lane, N)?;
        let slot = ranges
            .into_iter()
            .flatten()
            .find(|index| self.jobs[*index].is_none() && self.reservations[*index] == 0)
            .ok_or(WorkError::Full)?;
        let sequence = self.next;
        self.next = sequence.checked_add(1).ok_or(WorkError::Exhausted)?;
        self.jobs[slot] = Some(Job {
            callback,
            context: context as usize,
            lane,
            sequence,
            running: false,
            requirement,
        });
        Ok(sequence)
    }
    pub(super) fn completed(&self, sequence: u64) -> Result<bool, WorkError> {
        if sequence == 0 || sequence >= self.next {
            return Err(WorkError::InvalidTicket);
        }
        Ok(!self
            .jobs
            .iter()
            .flatten()
            .any(|job| job.sequence == sequence))
    }
    #[cfg(target_arch = "x86_64")]
    pub(super) fn select_job(
        &mut self,
        lane: Lane,
        geometry: stack_requirement::Geometry,
        abandoned: bool,
    ) -> Result<Option<(usize, Job)>, WorkError> {
        if self.poisoned || abandoned {
            return Err(WorkError::Closed);
        }
        let next = self
            .jobs
            .iter()
            .enumerate()
            .filter_map(|(i, j)| {
                j.filter(|j| {
                    self.reservations[i] == 0
                        && j.lane == lane
                        && !j.running
                        && stack_requirement::compatible(j.requirement, geometry)
                })
                .map(|j| (i, j))
            })
            .min_by_key(|(_, j)| j.sequence);
        if let Some((i, _)) = next {
            self.jobs[i].as_mut().expect("selected slot").running = true;
        }
        Ok(next)
    }
    /// Per lane, the jobs [`Self::select_job`] could pick for some stack:
    /// queued, not running and not reserved. One pass over the slots.
    #[cfg(target_arch = "x86_64")]
    pub(super) fn waiting_lanes(&self) -> [u16; LANES] {
        let mut waiting = [0u16; LANES];
        for (i, job) in self.jobs.iter().enumerate() {
            if let Some(job) = job
                && !job.running
                && self.reservations[i] == 0
            {
                let count = &mut waiting[lane_index(job.lane)];
                *count = count.saturating_add(1);
            }
        }
        waiting
    }
    /// [`Self::select_job`] that skips the slot scan when `waiting` (from
    /// [`Self::waiting_lanes`], kept current by this call) shows no job of
    /// `lane`. It answers exactly as `select_job` would: with no waiting
    /// job that scan finds none, and refusals still reach it.
    #[cfg(target_arch = "x86_64")]
    pub(super) fn select_waiting_job(
        &mut self,
        waiting: &mut [u16; LANES],
        lane: Lane,
        geometry: stack_requirement::Geometry,
        abandoned: bool,
    ) -> Result<Option<(usize, Job)>, WorkError> {
        let count = &mut waiting[lane_index(lane)];
        if *count == 0 && !self.poisoned && !abandoned {
            return Ok(None);
        }
        step_counts::add(&step_counts::STEP_COUNTS.scans, 1);
        let selected = self.select_job(lane, geometry, abandoned)?;
        if selected.is_some() {
            *count = count.saturating_sub(1);
        }
        Ok(selected)
    }
    #[cfg(target_arch = "x86_64")]
    pub(super) fn finish_job(
        &mut self,
        slot: usize,
        sequence: u64,
        released_stack: Option<ReleasedStackGeometry>,
    ) -> Result<(), WorkError> {
        if self.reservations.get(slot).is_none_or(|nonce| *nonce != 0) {
            return Err(WorkError::corrupt());
        }
        if !self
            .jobs
            .get(slot)
            .and_then(|j| *j)
            .is_some_and(|j| j.sequence == sequence && j.running)
        {
            self.poisoned = true;
            return Err(WorkError::corrupt());
        }
        #[cfg(target_arch = "x86_64")]
        if self
            .layout
            .admission(Lane::General, N)?
            .into_iter()
            .flatten()
            .any(|candidate| candidate == slot)
        {
            // Ledger only. No sink/callback runs on this completing Task/IF0.
            if let Some(shape) = released_stack {
                self.resources.reservation_released(shape)?;
            } else {
                self.resources.released()?;
            }
        }
        self.jobs[slot] = None;
        Ok(())
    }
}

#[cfg(all(test, target_arch = "x86_64"))]
mod selection_tests {
    extern crate std;
    use super::*;
    use std::vec::Vec;

    unsafe extern "C" fn job(_: *mut c_void) {}

    /// One idle task's selection in the recorded mix.
    type Record = (usize, Result<Option<(usize, u64)>, WorkError>);

    fn geometry() -> stack_requirement::Geometry {
        stack_requirement::Geometry::from_ranges(&(0x20000..0x40000), &(0x1f000..0x20000))
    }

    /// Run one recorded mix of admissions, reservations, completions and
    /// idle-task selections, with the per-task scan (`counted` false) or
    /// the counted scan.
    fn run(counted: bool) -> Vec<Record> {
        const SLOTS: usize = 64;
        const LANES_USED: [Lane; 4] =
            [Lane::General, Lane::Notify, Lane::General, Lane::Foreground];
        let mut state = State::<SLOTS>::new();
        let mut seed = 0x2545_f491_4f6c_dd1du64;
        let mut random = move |bound: u64| {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed % bound
        };
        // Twenty idle tasks over the lanes, as the dispatcher's task rows.
        let tasks: Vec<Lane> = (0..20).map(|i| LANES_USED[i % LANES_USED.len()]).collect();
        let mut busy: Vec<Option<usize>> = std::vec![None; tasks.len()];
        let mut records = Vec::new();
        for round in 0..400u64 {
            for _ in 0..random(6) {
                let lane = LANES_USED[random(4) as usize];
                // Full refusals are part of the mix and identical in both runs.
                let _ = state.enqueue(lane, job, core::ptr::null_mut(), None, false);
            }
            // Reserve or release an empty slot, as owned reservations do.
            let slot = random(SLOTS as u64) as usize;
            if state.jobs[slot].is_none() {
                state.reservations[slot] = if state.reservations[slot] == 0 {
                    round + 1
                } else {
                    0
                };
            }
            // Complete some running jobs.
            for owned in &mut busy {
                if let Some(slot) = *owned
                    && random(3) == 0
                {
                    state.jobs[slot] = None;
                    *owned = None;
                }
            }
            if round == 390 {
                state.poisoned = true;
            }
            let mut waiting = state.waiting_lanes();
            let cursor = round as usize % tasks.len();
            for n in 0..tasks.len() {
                let task = (cursor + n) % tasks.len();
                if busy[task].is_some() {
                    continue;
                }
                let selected = if counted {
                    state.select_waiting_job(&mut waiting, tasks[task], geometry(), false)
                } else {
                    state.select_job(tasks[task], geometry(), false)
                };
                if let Ok(Some((slot, _))) = selected {
                    busy[task] = Some(slot);
                }
                let failed = selected.is_err();
                records.push((
                    task,
                    selected.map(|job| job.map(|(slot, job)| (slot, job.sequence))),
                ));
                if failed {
                    break;
                }
            }
        }
        records
    }

    #[test]
    fn counted_selection_gives_every_task_the_same_job_as_the_per_task_scan() {
        let scanned = run(false);
        let counted = run(true);
        assert!(
            scanned
                .iter()
                .filter(|(_, job)| matches!(job, Ok(Some(_))))
                .count()
                > 500
        );
        assert!(scanned.iter().any(|(_, job)| matches!(job, Ok(None))));
        assert!(scanned.iter().any(|(_, job)| job.is_err()));
        assert_eq!(scanned, counted);
    }

    #[test]
    fn waiting_lanes_counts_only_free_unreserved_jobs() {
        let mut state = State::<8>::new();
        for lane in [Lane::General, Lane::General, Lane::Notify] {
            state
                .enqueue(lane, job, core::ptr::null_mut(), None, false)
                .unwrap();
        }
        let mut waiting = state.waiting_lanes();
        assert_eq!(waiting[lane_index(Lane::General)], 2);
        assert_eq!(waiting[lane_index(Lane::Notify)], 1);
        assert!(
            state
                .select_waiting_job(&mut waiting, Lane::General, geometry(), false)
                .unwrap()
                .is_some()
        );
        assert_eq!(waiting, state.waiting_lanes());
        assert!(matches!(
            state.select_waiting_job(&mut waiting, Lane::Service, geometry(), false),
            Ok(None)
        ));
        assert!(matches!(
            state.select_waiting_job(&mut waiting, Lane::Service, geometry(), true),
            Err(WorkError::Closed)
        ));
    }
}
