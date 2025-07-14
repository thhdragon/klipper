// src/move_queue.rs
#![cfg_attr(not(test), no_std)]

use crate::stepper::TrapezoidalMove;
use heapless::VecDeque;

// Define the capacity of the move queue. 16 is a reasonable starting point.
const QUEUE_CAPACITY: usize = 16;

/// A queue to hold upcoming motion commands.
pub struct MoveQueue {
    /// A double-ended queue to store TrapezoidalMove structs.
    moves: VecDeque<TrapezoidalMove, QUEUE_CAPACITY>,
}

impl MoveQueue {
    /// Creates a new, empty MoveQueue.
    pub fn new() -> Self {
        Self {
            moves: VecDeque::new(),
        }
    }

    /// Adds a move to the back of the queue.
    /// Returns an error if the queue is full.
    pub fn add_move(&mut self, mov: TrapezoidalMove) -> Result<(), &'static str> {
        self.moves.push_back(mov)
            .map_err(|_| "Move queue full")
    }

    /// Removes and returns the move from the front of the queue.
    /// Returns None if the queue is empty.
    pub fn pop_current_move(&mut self) -> Option<TrapezoidalMove> {
        self.moves.pop_front()
    }

    /// Returns a reference to the move at the front of the queue without removing it.
    /// This is the "current" move being planned.
    pub fn get_current_move(&self) -> Option<&TrapezoidalMove> {
        self.moves.front()
    }

    /// Returns a reference to the move *after* the current one, for look-ahead.
    /// Returns None if there is no next move.
    pub fn get_next_move(&self) -> Option<&TrapezoidalMove> {
        self.moves.get(1) // Get the element at index 1
    }

    /// Checks if the queue is empty.
    pub fn is_empty(&self) -> bool {
        self.moves.is_empty()
    }

    /// Returns the number of moves currently in the queue.
    #[allow(dead_code)]
    pub fn len(&self) -> usize {
        self.moves.len()
    }
}
