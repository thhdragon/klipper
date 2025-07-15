// src/sched.rs
#![cfg_attr(not(test), no_std)]

use crate::hal::{
    Scheduler as KlipperSchedulerTrait,
    Timer as KlipperHalTimer,
    StepEventResult
};
use crate::rp2040_hal_impl::timer::Rp2040Timer; // Not directly used here, but Timer trait is

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
    master_timer: Rp2040Timer<MasterAlarm>, // Scheduler's own timer
    raw_rp_timer: RpHalTimer,
    task_dispatcher: fn(u32),
}

impl<MasterAlarm: Alarm> SchedulerState<MasterAlarm> {
    pub fn new(
        master_timer_alarm: MasterAlarm,
        raw_rp_timer: RpHalTimer,
        master_timer_callback: fn(&mut Rp2040Timer<MasterAlarm>) -> StepEventResult,
        task_dispatcher: fn(u32),
        // Add ID for the master_timer itself, though not strictly needed by scheduler logic for it
        master_timer_id: u32,
    ) -> Self {
        let master_timer = Rp2040Timer::new(master_timer_alarm, master_timer_callback, master_timer_id);
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
            // defmt::trace!("Scheduler: Next task ID {} at {}. Rescheduling master.", next_task.id, next_task.waketime);
            self.master_timer.set_waketime(next_task.waketime);
        } else {
            // defmt::trace!("Scheduler: No tasks. Master timer not rescheduled.");
            // Optionally explicitly disarm master_timer if it has such a method and is needed.
            // For now, not calling set_waketime means its IRQ won't re-fire.
        }
    }

    // Public method for explicit task scheduling or rescheduling.
    pub fn schedule_task(&mut self, task_id: u32, waketime: u32) {
        // Remove any existing task with the same ID
        let mut temp_tasks = Vec::<ScheduledTask, MAX_SCHEDULED_TIMERS>::new();
        let mut task_updated = false;
        while let Some(task) = self.tasks.pop() {
            if task.id == task_id {
                // Update existing task's waketime if found, then re-add
                // This is effectively what add_timer should do.
                // For simplicity of BinaryHeap, we remove and re-add.
                // A more advanced heap might offer decrease_key.
                // For now, just ensure it's not duplicated by removing.
                task_updated = true; // Mark that we found and removed it
            } else {
                temp_tasks.push(task).ok(); // Keep other tasks
            }
        }
        // Push back other tasks
        for task in temp_tasks {
            self.tasks.push(task).ok();
        }

        // Add the new/updated task
        let new_task = ScheduledTask { id: task_id, waketime };
        if self.tasks.push(new_task).is_ok() {
            if task_updated {
                defmt::trace!("Scheduler: Updated task ID {} to waketime {}", task_id, waketime);
            } else {
                defmt::trace!("Scheduler: Scheduled new task ID {} for waketime {}", task_id, waketime);
            }
            self.reschedule_master_timer_interrupt();
        } else {
            defmt::error!("Scheduler: Task queue full! Cannot schedule task ID {}", task_id);
        }
    }
}

impl<MasterAlarm: Alarm> KlipperSchedulerTrait for SchedulerState<MasterAlarm> {
    fn add_timer(&mut self, timer_ref: &mut dyn KlipperHalTimer) {
        let task_id = timer_ref.get_id(); // Get the actual ID from the timer
        let waketime = timer_ref.get_waketime();

        defmt::debug!("Scheduler (add_timer): Updating/Adding Task ID {} to waketime {}.", task_id, waketime);
        self.schedule_task(task_id, waketime); // Use the public method to handle heap update
    }

    fn delete_timer(&mut self, timer_to_delete: &mut dyn KlipperHalTimer) {
        let task_id_to_remove = timer_to_delete.get_id(); // Get the actual ID

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
        } else {
            defmt::warn!("Scheduler (delete_timer): Task ID {} not found in queue.", task_id_to_remove);
        }
    }

    fn read_time(&self) -> u32 {
        self.raw_rp_timer.get_counter_low()
    }

    fn get_clock_freq(&self) -> u32 {
        1_000_000
    }
}
