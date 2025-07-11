// src/sched.rs
#![cfg_attr(not(test), no_std)]

use crate::hal::{
    Scheduler as KlipperSchedulerTrait,
    Timer as KlipperHalTimer,
    StepEventResult
};
use crate::rp2040_hal_impl::timer::Rp2040Timer;

use rp2040_hal::timer::{Alarm, Timer as RpHalTimer};
use heapless::binary_heap::{BinaryHeap, Max};
use heapless::Vec;

use defmt;

const MAX_SCHEDULED_TIMERS: usize = 8;

#[derive(Copy, Clone, Debug)]
pub struct ScheduledTask {
    pub id: u32,
    pub waketime: u32,
}

impl PartialEq for ScheduledTask {
    fn eq(&self, other: &Self) -> bool {
        self.waketime == other.waketime && self.id == other.id
    }
}
impl Eq for ScheduledTask {}

impl PartialOrd for ScheduledTask {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for ScheduledTask {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        other.waketime.cmp(&self.waketime)
            .then_with(|| self.id.cmp(&other.id))
    }
}

pub struct SchedulerState<MasterAlarm: Alarm> {
    tasks: BinaryHeap<ScheduledTask, Max, MAX_SCHEDULED_TIMERS>,
    master_timer: Rp2040Timer<MasterAlarm>,
    raw_rp_timer: RpHalTimer,
    task_dispatcher: fn(u32), // Function pointer for dispatching tasks
}

impl<MasterAlarm: Alarm> SchedulerState<MasterAlarm> {
    pub fn new(
        master_timer_alarm: MasterAlarm,
        raw_rp_timer: RpHalTimer,
        master_timer_callback: fn(&mut Rp2040Timer<MasterAlarm>) -> StepEventResult,
        task_dispatcher: fn(u32), // Added dispatcher
    ) -> Self {
        let master_timer = Rp2040Timer::new(master_timer_alarm, master_timer_callback);
        Self {
            tasks: BinaryHeap::new(),
            master_timer,
            raw_rp_timer,
            task_dispatcher,
        }
    }

    pub fn service_master_timer(&mut self) {
        let now = self.raw_rp_timer.get_counter_low();
        let mut to_dispatch_ids = Vec::<u32, MAX_SCHEDULED_TIMERS>::new();

        while let Some(task) = self.tasks.peek() {
            if task.waketime <= now {
                let due_task = self.tasks.pop().unwrap();
                to_dispatch_ids.push(due_task.id).ok();
            } else {
                break;
            }
        }

        for task_id in to_dispatch_ids {
            defmt::debug!("Scheduler: Dispatching task ID {} via fn ptr", task_id);
            (self.task_dispatcher)(task_id);
        }

        self.reschedule_master_timer_interrupt();
    }

    fn reschedule_master_timer_interrupt(&mut self) {
        if let Some(next_task) = self.tasks.peek() {
            self.master_timer.set_waketime(next_task.waketime);
        }
    }

    pub fn schedule_task(&mut self, task_id: u32, waketime: u32) {
        let mut temp_heap = BinaryHeap::<ScheduledTask, Max, MAX_SCHEDULED_TIMERS>::new();
        while let Some(task) = self.tasks.pop() {
            if task.id != task_id {
                temp_heap.push(task).ok();
            }
        }
        self.tasks = temp_heap;

        let task = ScheduledTask { id: task_id, waketime };
        if self.tasks.push(task).is_ok() {
            self.reschedule_master_timer_interrupt();
        } else {
            defmt::error!("Scheduler: Task queue full! Cannot schedule task ID {}", task_id);
        }
    }
}

impl<MasterAlarm: Alarm> KlipperSchedulerTrait for SchedulerState<MasterAlarm> {
    fn add_timer(&mut self, timer_ref: &mut dyn KlipperHalTimer) {
        // HACK/Simplification: Assumes ID 0 for any timer passed for now.
        // A proper system would need `timer_ref` to provide its ID.
        let task_id_for_timer_ref = 0;
        let waketime = timer_ref.get_waketime();
        defmt::debug!("Scheduler (add_timer): Task ID {} waketime {}. Re-evaluating.", task_id_for_timer_ref, waketime);
        self.schedule_task(task_id_for_timer_ref, waketime);
    }

    fn delete_timer(&mut self, _timer_to_delete: &mut dyn KlipperHalTimer) {
        // HACK/Simplification: Assumes ID 0.
        let task_id_to_remove = 0;

        let mut temp_heap = BinaryHeap::<ScheduledTask, Max, MAX_SCHEDULED_TIMERS>::new();
        let mut found = false;
        while let Some(task) = self.tasks.pop() {
            if task.id != task_id_to_remove {
                temp_heap.push(task).ok();
            } else {
                found = true;
            }
        }
        self.tasks = temp_heap;

        if found {
            defmt::debug!("Scheduler (delete_timer): Removed task ID {}. Re-evaluating.", task_id_to_remove);
            self.reschedule_master_timer_interrupt();
        }
    }

    fn read_time(&self) -> u32 {
        self.raw_rp_timer.get_counter_low()
    }

    fn get_clock_freq(&self) -> u32 {
        1_000_000
    }
}
