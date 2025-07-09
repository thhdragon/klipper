// klipper_rust_port/src/trapq.rs

#![cfg_attr(not(test), no_std)] // Apply no_std only when not testing

#[derive(Debug, Copy, Clone, PartialEq, Default)]
pub struct Coord {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Coord {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Coord { x, y, z }
    }
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct Move {
    pub print_time: f64,
    pub move_t: f64,
    pub start_v: f64,
    pub half_accel: f64, // half of acceleration for this move segment
    pub start_pos: Coord,
    pub axes_r: Coord, // Direction ratios (normalized vector * distance)

    // `node: list_node` from C is omitted for now.
    // Will be handled by collection management within TrapQ or if an intrusive list is explicitly needed.
}

impl Move {
    pub fn new(
        print_time: f64,
        move_t: f64,
        start_v: f64,
        half_accel: f64,
        start_pos: Coord,
        axes_r: Coord,
    ) -> Self {
        Move {
            print_time,
            move_t,
            start_v,
            half_accel,
            start_pos,
            axes_r,
        }
    }

    #[cfg(feature = "alloc")]
    mod trapq_set_position_tests {
        use super::*;

        fn non_null_move(print_time: f64, move_t: f64, start_pos: Coord) -> Move {
            Move::new(print_time, move_t, 1.0, 0.5, start_pos, Coord::new(1.0,0.0,0.0))
        }

        #[test]
        fn set_position_empty_queue() {
            let mut tq = new_test_tq();
            let pos = Coord::new(10.0, 20.0, 30.0);
            tq.set_position(5.0, pos);

            assert!(tq.moves.is_empty());
            assert_eq!(tq.history.len(), 1);
            let marker = &tq.history[0];
            assert_approx_eq!(f64, marker.print_time, 5.0);
            assert_approx_eq!(f64, marker.move_t, 0.0);
            assert_eq!(marker.start_pos, pos);
            assert_eq!(marker.axes_r, Coord::new(0.0,0.0,0.0));
        }

        #[test]
        fn set_position_flushes_moves_and_adds_marker() {
            let mut tq = new_test_tq();
            tq.moves.push(non_null_move(0.0, 1.0, Coord::new(0.,0.,0.)));
            tq.moves.push(non_null_move(1.0, 1.0, Coord::new(1.,0.,0.)));

            let set_pos_time = 0.5;
            let new_pos = Coord::new(10.0, 0.0, 0.0);
            tq.set_position(set_pos_time, new_pos);

            assert!(tq.moves.is_empty(), "Moves queue should be empty after set_position");

            // Finalize_moves(LARGE_TIME, 0.0) is called first.
            // m(0,1) ends 1.0. m(1,1) ends 2.0. Both expire.
            // History before pruning: [m(1,1) at pos(1,0,0), m(0,1) at pos(0,0,0)]
            // clear_history_time = 0.0 means all but head (m(1,1)) are pruned.
            // So, history before set_pos pruning: [m(1,1) at pos(1,0,0)]

            // Now, set_position's pruning logic for history:
            // Head of history is m(1,1) (print_time 1.0). set_pos_time is 0.5.
            // head_hist_move.print_time (1.0) is NOT < set_pos_time (0.5). So, remove it.
            // History becomes empty.
            // Then marker is added.

            assert_eq!(tq.history.len(), 1, "History should contain only the new marker");
            let marker = &tq.history[0];
            assert_approx_eq!(f64, marker.print_time, set_pos_time);
            assert_eq!(marker.start_pos, new_pos);
        }

        #[test]
        fn set_position_truncates_history_move() {
            let mut tq = new_test_tq();
            // History (head to tail): [m(1.0, 2.0, pos2), m(0.0, 1.0, pos1)]
            // m_hist_1: pt=0.0, mt=1.0, end_t=1.0, pos=(0,0,0)
            // m_hist_0: pt=1.0, mt=2.0, end_t=3.0, pos=(1,0,0)
            let pos1 = Coord::new(0.,0.,0.);
            let pos2 = Coord::new(1.,0.,0.);
            tq.history.push(non_null_move(0.0, 1.0, pos1)); // Tail
            tq.history.push(non_null_move(1.0, 2.0, pos2)); // Head

            // Current history (idx 0 is head):
            // 0: Move { print_time: 1.0, move_t: 2.0, ... } // ends at 3.0
            // 1: Move { print_time: 0.0, move_t: 1.0, ... } // ends at 1.0

            let set_pos_time = 1.5; // This time is within m_hist_0
            let new_pos = Coord::new(10.0, 0.0, 0.0);
            tq.set_position(set_pos_time, new_pos);

            // finalize_moves(LARGE, 0.0) is called:
            // History: [m(1,2,pos2)] (m(0,1,pos1) is pruned as it's not head and clear_history_time=0)

            // set_position pruning:
            // Head is m(1,2,pos2). print_time 1.0 < set_pos_time 1.5.
            // print_time (1.0) + move_t (2.0) = 3.0 > set_pos_time (1.5). So truncate.
            // new move_t = 1.5 - 1.0 = 0.5.
            // History becomes: [truncated_m(1,0.5,pos2)]
            // Then marker is added at head.
            // History: [marker, truncated_m(1,0.5,pos2)]

            assert_eq!(tq.history.len(), 2);

            let marker = &tq.history[0];
            assert_approx_eq!(f64, marker.print_time, set_pos_time);
            assert_eq!(marker.start_pos, new_pos);

            let truncated_move = &tq.history[1];
            assert_approx_eq!(f64, truncated_move.print_time, 1.0);
            assert_approx_eq!(f64, truncated_move.move_t, 0.5); // Original 2.0, truncated
            assert_eq!(truncated_move.start_pos, pos2);
        }

        #[test]
        fn set_position_keeps_history_move_before_set_time() {
            let mut tq = new_test_tq();
            // History (head to tail): [m(1.0,1.0,pos2), m(0.0,1.0,pos1)]
             let pos1 = Coord::new(0.,0.,0.);
             let pos2 = Coord::new(1.,0.,0.);
            tq.history.push(non_null_move(0.0, 1.0, pos1)); // ends 1.0
            tq.history.push(non_null_move(1.0, 1.0, pos2)); // ends 2.0 (Head)

            let set_pos_time = 2.5; // After all history moves
            let new_pos = Coord::new(10.0, 0.0, 0.0);
            tq.set_position(set_pos_time, new_pos);

            // finalize_moves(LARGE, 0.0) -> history: [m(1,1,pos2)]
            // set_position pruning:
            // Head is m(1,1,pos2). print_time 1.0 < set_pos_time 2.5.
            // print_time (1.0) + move_t (1.0) = 2.0 <= set_pos_time (2.5). Not truncated. Kept.
            // History: [m(1,1,pos2)]
            // Then marker added at head.
            // History: [marker, m(1,1,pos2)]

            assert_eq!(tq.history.len(), 2);
            let marker = &tq.history[0];
            assert_approx_eq!(f64, marker.print_time, set_pos_time);
            assert_eq!(marker.start_pos, new_pos);

            let kept_move = &tq.history[1];
            assert_approx_eq!(f64, kept_move.print_time, 1.0);
            assert_approx_eq!(f64, kept_move.move_t, 1.0); // Not truncated
            assert_eq!(kept_move.start_pos, pos2);
        }
         #[test]
        fn set_position_removes_history_moves_after_set_time() {
            let mut tq = new_test_tq();
            // History (head to tail): [m(2.0,1.0,pos3), m(1.0,1.0,pos2), m(0.0,1.0,pos1)]
            let pos1 = Coord::new(0.,0.,0.);
            let pos2 = Coord::new(1.,0.,0.);
            let pos3 = Coord::new(2.,0.,0.);
            tq.history.push(non_null_move(0.0, 1.0, pos1)); // ends 1.0
            tq.history.push(non_null_move(1.0, 1.0, pos2)); // ends 2.0
            tq.history.push(non_null_move(2.0, 1.0, pos3)); // ends 3.0 (Head)

            let set_pos_time = 0.5; // Before all history moves
            let new_pos = Coord::new(10.0, 0.0, 0.0);
            tq.set_position(set_pos_time, new_pos);

            // finalize_moves(LARGE,0.0) -> history: [m(2,1,pos3)]
            // set_position pruning:
            // Head is m(2,1,pos3). print_time 2.0 is NOT < set_pos_time 0.5. Remove.
            // History becomes empty.
            // Marker added.
            // History: [marker]

            assert_eq!(tq.history.len(), 1);
            let marker = &tq.history[0];
            assert_approx_eq!(f64, marker.print_time, set_pos_time);
            assert_eq!(marker.start_pos, new_pos);
        }
    }

    #[cfg(feature = "alloc")]
    mod trapq_finalize_moves_tests {
        use super::*;

        fn non_null_move(print_time: f64, move_t: f64) -> Move {
            Move::new(print_time, move_t, 1.0, 0.5, Coord::default(), Coord::default())
        }
        fn null_move_obj(print_time: f64, move_t: f64) -> Move { // Renamed to avoid conflict
            Move::new(print_time, move_t, 0.0, 0.0, Coord::default(), Coord::default())
        }

        #[test]
        fn finalize_empty_queue() {
            let mut tq = new_test_tq();
            tq.finalize_moves(10.0, 5.0);
            assert!(tq.moves.is_empty());
            assert!(tq.history.is_empty());
        }

        #[test]
        fn finalize_all_moves_expired_to_history() {
            let mut tq = new_test_tq();
            tq.moves.push(non_null_move(0.0, 1.0)); // Ends at 1.0
            tq.moves.push(non_null_move(1.0, 1.0)); // Ends at 2.0

            tq.finalize_moves(2.5, 0.0); // Expire up to 2.5, clear history before time 0

            assert!(tq.moves.is_empty());
            assert_eq!(tq.history.len(), 2);
            // Moves are added to head of history, so order is reversed
            assert_approx_eq!(f64, tq.history[0].print_time, 1.0); // m2
            assert_approx_eq!(f64, tq.history[1].print_time, 0.0); // m1
        }

        #[test]
        fn finalize_null_moves_are_discarded() {
            let mut tq = new_test_tq();
            tq.moves.push(null_move_obj(0.0, 1.0));
            tq.moves.push(non_null_move(1.0, 1.0));
            tq.moves.push(null_move_obj(2.0, 1.0));

            tq.finalize_moves(3.5, 0.0);

            assert!(tq.moves.is_empty());
            assert_eq!(tq.history.len(), 1); // Only non-null move goes to history
            assert_approx_eq!(f64, tq.history[0].print_time, 1.0);
        }

        #[test]
        fn finalize_some_moves_expired() {
            let mut tq = new_test_tq();
            let m1 = non_null_move(0.0, 1.0); // Ends 1.0
            let m2 = non_null_move(1.0, 1.0); // Ends 2.0
            let m3 = non_null_move(2.0, 1.0); // Ends 3.0
            tq.moves.push(m1.clone());
            tq.moves.push(m2.clone());
            tq.moves.push(m3.clone());

            tq.finalize_moves(1.5, 0.0); // Expire m1 (ends at 1.0 < 1.5)

            assert_eq!(tq.moves.len(), 2);
            assert_eq!(tq.moves[0], m2); // m2 is now at the head of moves
            assert_eq!(tq.moves[1], m3);

            assert_eq!(tq.history.len(), 1);
            assert_eq!(tq.history[0], m1);
        }

        #[test]
        fn finalize_history_pruning_none_old_enough() {
            let mut tq = new_test_tq();
            tq.history.push(non_null_move(0.0, 1.0)); // Ends 1.0
            tq.history.push(non_null_move(1.0, 1.0)); // Ends 2.0 (head of history)
                                                      // History: [m(1.0,1.0), m(0.0,1.0)]

            tq.finalize_moves(0.0, 0.5); // clear_history_time = 0.5. Both moves end after this.

            assert_eq!(tq.history.len(), 2);
        }

        #[test]
        fn finalize_history_pruning_one_old_enough_but_is_head() {
            let mut tq = new_test_tq();
            // History will be [m(0.0,1.0)]
            tq.history.push(non_null_move(0.0, 1.0)); // Ends 1.0. This is the head.

            // clear_history_time = 1.5. Move ends before this, so it's "old".
            // But it's the only item (head), so it should be kept.
            tq.finalize_moves(0.0, 1.5);

            assert_eq!(tq.history.len(), 1);
            assert_approx_eq!(f64, tq.history[0].print_time, 0.0);
        }

        #[test]
        fn finalize_history_pruning_some_old_enough_from_tail() {
            let mut tq = new_test_tq();
            // History (head to tail): [m(2,1), m(1,1), m(0,1)]
            // Times (end):              3.0      2.0      1.0
            tq.history.push(non_null_move(0.0, 1.0)); // Tail: Ends 1.0
            tq.history.push(non_null_move(1.0, 1.0)); // Middle: Ends 2.0
            tq.history.push(non_null_move(2.0, 1.0)); // Head: Ends 3.0

            // clear_history_time = 1.5.
            // m(0,1) ends at 1.0 <= 1.5, so it's old and not head. Remove.
            // m(1,1) ends at 2.0 > 1.5. Keep. Stop.
            tq.finalize_moves(0.0, 1.5);

            assert_eq!(tq.history.len(), 2);
            // Expected history (head to tail): [m(2,1), m(1,1)]
            assert_approx_eq!(f64, tq.history[0].print_time, 2.0);
            assert_approx_eq!(f64, tq.history[1].print_time, 1.0);
        }

        #[test]
        fn finalize_history_pruning_all_but_head_old_enough() {
            let mut tq = new_test_tq();
            // History (head to tail): [m(2,1), m(1,1), m(0,1)]
            tq.history.push(non_null_move(0.0, 1.0));
            tq.history.push(non_null_move(1.0, 1.0));
            tq.history.push(non_null_move(2.0, 1.0));

            // clear_history_time = 2.5
            // m(0,1) ends 1.0 <= 2.5. Remove.
            // m(1,1) ends 2.0 <= 2.5. Remove.
            // m(2,1) is head. Keep.
            tq.finalize_moves(0.0, 2.5);

            assert_eq!(tq.history.len(), 1);
            assert_approx_eq!(f64, tq.history[0].print_time, 2.0); // Only head remains
        }
    }

    /// Calculate the distance covered during this move up to a given move_time.
    /// Equivalent to C: double move_get_distance(struct move *m, double move_time)
    pub fn get_distance(&self, time_into_move: f64) -> f64 {
        // Clamp time_into_move to the duration of this move segment
        let t = time_into_move.max(0.0).min(self.move_t);
        self.start_v * t + self.half_accel * t * t
    }

    /// Calculate the coordinate at a given time into this move.
    /// Equivalent to C: struct coord move_get_coord(struct move *m, double move_time)
    pub fn get_coord(&self, time_into_move: f64) -> Coord {
        let dist = self.get_distance(time_into_move);
        Coord {
            x: self.start_pos.x + self.axes_r.x * dist,
            y: self.start_pos.y + self.axes_r.y * dist,
            z: self.start_pos.z + self.axes_r.z * dist,
        }
    }
}


// For now, TrapQ will use Vec for simplicity.
// In a strict no_std environment without an allocator, this would need
// to be a fixed-size array or a custom heapless collection.
// The use of `Vec` implies `alloc` feature for `no_std` or running with `std` (like in tests).
#[cfg(feature = "alloc")]
use alloc::vec::Vec;

#[derive(Debug, Clone, PartialEq, Default)]
#[cfg_attr(feature = "std", derive(serde::Serialize, serde::Deserialize))]
pub struct PullMove {
    pub print_time: f64,
    pub move_t: f64,
    pub start_v: f64,
    pub accel: f64,
    pub start_pos: Coord, // Using the existing Coord struct for x,y,z
    pub axes_r: Coord,    // Using the existing Coord struct for x_r,y_r,z_r
}

#[derive(Debug, Clone, Default)]
pub struct TrapQ {
    #[cfg(feature = "alloc")]
    pub moves: Vec<Move>,
    #[cfg(feature = "alloc")]
    pub history: Vec<Move>,

    // If no alloc feature, these would be fixed-size arrays or custom list structures.
    // For example, using an array and a count:
    // pub moves: [Option<Move>; MAX_MOVES],
    // pub moves_count: usize,
    // pub history: [Option<Move>; MAX_HISTORY_MOVES],
    // pub history_count: usize,
}

// C's NEVER_TIME for sentinel comparisons. Not directly used with Vec but good for reference.
// const NEVER_TIME: f64 = 9999999999999999.9;
const MAX_NULL_MOVE_DURATION: f64 = 1.0;


impl TrapQ {
    #[cfg(feature = "alloc")]
    pub fn new() -> Self {
        // The C version trapq_alloc also initializes sentinels.
        // Vec doesn't use sentinels in the same way.
        // list_init(&tq->moves) and list_init(&tq->history) are analogous to Vec::new().
        TrapQ {
            moves: Vec::new(),
            history: Vec::new(),
        }
    }

    #[cfg(not(feature = "alloc"))]
    pub fn new_no_alloc() -> Self {
        // Placeholder if we need a non-alloc version later.
        TrapQ {
            // This would require fixed-size arrays or similar.
        }
    }

    /// Internal helper corresponding to C's trapq_add_move.
    /// Adds a single move segment. Handles inserting a null move if there's a time gap.
    #[cfg(feature = "alloc")]
    fn add_move_segment(&mut self, m: Move) {
        if let Some(prev_move) = self.moves.last() {
            if prev_move.print_time + prev_move.move_t < m.print_time {
                // Add a null move to fill time gap
                let mut null_move_print_time = prev_move.print_time + prev_move.move_t;
                // C logic: if (!prev->print_time && m->print_time > MAX_NULL_MOVE)
                // This means if prev_move is effectively the "head sentinel" (print_time 0)
                // and the gap is large, limit the first null move.
                // For Vec, if self.moves is empty, prev_move wouldn't exist.
                // If prev_move.print_time == 0.0 (assuming this is how we'd model the initial state before any real moves)
                // and m.print_time is large.
                if prev_move.print_time == 0.0 && prev_move.move_t == 0.0 && // Heuristic for "initial state"
                   m.print_time > MAX_NULL_MOVE_DURATION {
                    null_move_print_time = m.print_time - MAX_NULL_MOVE_DURATION;
                }

                let null_move = Move {
                    print_time: null_move_print_time,
                    move_t: m.print_time - null_move_print_time,
                    start_v: 0.0, // Null move has no velocity or acceleration
                    half_accel: 0.0,
                    // start_pos for null move should be end_pos of prev_move,
                    // which is also start_pos of current move `m` if axes_r is (0,0,0) for null_move.
                    // C code sets: null_move->start_pos = m->start_pos;
                    // This is fine if the null move has zero length in terms of axes_r.
                    start_pos: m.start_pos,
                    axes_r: Coord::new(0.0, 0.0, 0.0), // No displacement for a null move
                };
                // Ensure null_move.move_t is not negative if times are misaligned
                if null_move.move_t > 0.0 {
                    self.moves.push(null_move);
                }
            }
        } else {
            // No previous move, this is the first move being added.
            // C code might insert a null move from time 0 up to m.print_time - MAX_NULL_MOVE
            // if m.print_time is large.
            if m.print_time > MAX_NULL_MOVE_DURATION {
                 let null_move = Move {
                    print_time: 0.0, // Starts from time 0
                    move_t: m.print_time - MAX_NULL_MOVE_DURATION,
                    start_v: 0.0,
                    half_accel: 0.0,
                    start_pos: m.start_pos, // Position doesn't change
                    axes_r: Coord::new(0.0,0.0,0.0),
                };
                self.moves.push(null_move);
            }
        }
        self.moves.push(m);
    }


    /// Fill and add a move (accel, cruise, decel phases) to the trapezoid velocity queue.
    /// Corresponds to C: void trapq_append(...)
    #[cfg(feature = "alloc")]
    pub fn append(
        &mut self,
        mut print_time: f64,
        accel_t: f64,
        cruise_t: f64,
        decel_t: f64,
        mut start_pos: Coord, // Changed from individual floats to Coord
        axes_r: Coord,    // Changed from individual floats to Coord
        start_v: f64,
        cruise_v: f64,
        accel: f64,
    ) {
        if accel_t > 0.0 { // Use a small epsilon if checking for non-zero float time
            let m = Move {
                print_time,
                move_t: accel_t,
                start_v,
                half_accel: 0.5 * accel,
                start_pos,
                axes_r,
            };
            self.add_move_segment(m); // Use internal helper

            print_time += accel_t;
            // Calculate end position of this segment to be start_pos for the next
            // This was: start_pos = move_get_coord(m, accel_t);
            // but `m` is moved into add_move_segment. We need to re-calculate based on original m's fields.
            let end_dist = (start_v + (0.5 * accel) * accel_t) * accel_t;
            start_pos = Coord {
                x: start_pos.x + axes_r.x * end_dist,
                y: start_pos.y + axes_r.y * end_dist,
                z: start_pos.z + axes_r.z * end_dist,
            };
        }

        if cruise_t > 0.0 {
            let m = Move {
                print_time,
                move_t: cruise_t,
                start_v: cruise_v,
                half_accel: 0.0,
                start_pos,
                axes_r,
            };
            self.add_move_segment(m);

            print_time += cruise_t;
            let end_dist = cruise_v * cruise_t; // distance = v*t for cruise phase
            start_pos = Coord {
                x: start_pos.x + axes_r.x * end_dist,
                y: start_pos.y + axes_r.y * end_dist,
                z: start_pos.z + axes_r.z * end_dist,
            };
        }

        if decel_t > 0.0 {
            let m = Move {
                print_time,
                move_t: decel_t,
                start_v: cruise_v,
                half_accel: -0.5 * accel,
                start_pos,
                axes_r,
            };
            self.add_move_segment(m);
            // No need to update print_time or start_pos after the last segment
        }
    }

    /// Expire any moves older than `print_time` from the main `moves` queue
    /// to the `history` queue. Also, free very old moves from the history.
    /// Corresponds to C: void trapq_finalize_moves(...)
    #[cfg(feature = "alloc")]
    pub fn finalize_moves(&mut self, print_time: f64, clear_history_time: f64) {
        // Move expired moves from self.moves to self.history
        // In C, this iterates from head_sentinel.next.
        // With Vec, we can find the split point.
        let mut i = 0;
        while i < self.moves.len() {
            let m = &self.moves[i];
            if m.print_time + m.move_t > print_time {
                break; // This move and subsequent ones are not expired yet
            }
            i += 1;
        }

        // `i` is now the count of moves to transfer or the index of the first non-expired move.
        // These moves (0..i) are expired.
        let expired_moves: Vec<Move> = self.moves.drain(0..i).collect();

        for m_expired in expired_moves {
            // In C: if (m->start_v || m->half_accel) list_add_head(&m->node, &tq->history);
            // else free(m);
            // This means "null moves" (those with no velocity or accel) are freed directly.
            // Other moves are added to the history.
            if m_expired.start_v != 0.0 || m_expired.half_accel != 0.0 {
                // Add to the beginning of history (like list_add_head)
                self.history.insert(0, m_expired);
            } else {
                // If not added to history, it's effectively "freed" as it's removed from `moves`
                // and `m_expired` will be dropped here.
            }
        }

        // Free old moves from history list
        // C: iterate from last_entry of history and remove if older than clear_history_time
        //    and not the 'latest' (first entry) in history.
        if self.history.is_empty() {
            return;
        }

        // The C code has a peculiar condition: `m == latest` where `latest` is the first entry.
        // This means it doesn't remove the very first item in history, even if it's too old.
        // Let's try to replicate: retain elements if they are newer than clear_history_time OR if it's the first element.

        let latest_history_print_time = self.history.first().map_or(0.0, |m| m.print_time + m.move_t);

        self.history.retain(|m_hist| {
            let is_latest_equivalent = self.history.first().map_or(false, |first| {
                // Compare by content or a unique ID if available. Pointers in C made this easy.
                // For Vec<Move>, if Move is PartialEq, we can compare.
                // However, the C logic is `m == latest` (pointer comparison).
                // This means it protects the *current first element* of the history list.
                // So, if m_hist is the current first element during `retain`, it's kept.
                // This is tricky with `retain`. A simpler interpretation for `retain` might be:
                // keep if (m_hist.print_time + m_hist.move_t > clear_history_time)
                // The C code's `m == latest` check within the loop `list_last_entry` means it stops
                // before removing the head of the history queue.
                // So, we want to remove from the end of history, stopping if we hit the current head,
                // or if the move is new enough.
                // `retain` processes from the beginning.
                // Let's re-evaluate. C iterates from the *end* of history.
                // `list_last_entry(&tq->history, struct move, node);`
                // `if (m == latest || m->print_time + m->move_t > clear_history_time) break;`
                // `list_del(&m->node); free(m);`
                // This means it removes from the tail until it finds one that is either the head OR new enough.
                false // placeholder, this logic needs to be correct
            });

            // Corrected logic for pruning history:
            // We remove from the end of history (older moves) as long as:
            // 1. The history is not empty.
            // 2. The move to remove is NOT the head of the history list.
            // 3. The move IS older than clear_history_time.
            // This loop continues until one of these conditions fails.
            // This is easier done with a `while let Some(last_move) = self.history.last()` loop.
            // However, we must ensure not to remove the head if it's the only one left and too old.
            // C code: `struct move *latest = list_first_entry(&tq->history, struct move, node);`
            // `struct move *m = list_last_entry(&tq->history, struct move, node);`
            // `if (m == latest || ...)`
            // This means if history has only one element, `m == latest` is true, so it's not removed.

            // Let's try with `pop` from the end.
            while self.history.len() > 1 { // Ensure there's more than one element before considering removal of last.
                if let Some(last_move_in_history) = self.history.last() {
                    if last_move_in_history.print_time + last_move_in_history.move_t <= clear_history_time {
                        self.history.pop(); // Remove the last (oldest among those eligible for removal)
                    } else {
                        break; // Last move is new enough, stop
                    }
                } else {
                    break; // Should not happen if len > 1
                }
            }
            // After the loop, if history still has one element (len == 1),
            // that single element is the 'latest' and was protected by `len > 1`.
            // This matches the C logic of not removing `latest`.
        //}); // The retain call was incorrect, replaced by loop below.

        // Corrected loop for pruning history based on C logic:
        // Remove from tail, but don't remove the head of history.
        while self.history.len() > 1 {
            // Peek at the last element. If it's too old, pop it.
            // The check `m == latest` in C means if list has 1 element, it's not removed.
            // So we only pop if len > 1 and the last one is too old.
            if let Some(last_hist_move) = self.history.last() {
                if last_hist_move.print_time + last_hist_move.move_t > clear_history_time {
                    break; // This move and (by implication) earlier ones in history are new enough
                }
                // If we are here, the last_move is older than clear_history_time.
                // Since len > 1, this is not the head sentinel. So, remove it.
                self.history.pop();
            } else {
                break; // Should not happen if len > 1
            }
        }
    }

    /// Note a position change in the trapq history.
    /// This involves flushing all moves from the main queue, pruning history,
    /// and adding a new marker to the history.
    /// Corresponds to C: void trapq_set_position(...)
    #[cfg(feature = "alloc")]
    pub fn set_position(&mut self, print_time: f64, pos: Coord) {
        // Flush all moves from trapq:
        // C: trapq_finalize_moves(tq, NEVER_TIME, 0);
        // NEVER_TIME effectively means all moves are expired.
        // clear_history_time = 0 means all but the latest history item are cleared.
        const ARBITRARILY_LARGE_TIME: f64 = f64::MAX / 2.0; // Effectively "never"
        self.finalize_moves(ARBITRARILY_LARGE_TIME, 0.0);

        // Prune any moves in the trapq history that were interrupted by the new position.
        // C: while (!list_empty(&tq->history)) {
        //        struct move *m = list_first_entry(&tq->history, struct move, node);
        //        if (m->print_time < print_time) {
        //            if (m->print_time + m->move_t > print_time)
        //                m->move_t = print_time - m->print_time; // Truncate move
        //            break;
        //        }
        //        list_del(&m->node); free(m); // Remove fully future moves
        //    }
        // This means: iterate from head of history. If a move is fully after print_time, remove it.
        // If a move straddles print_time, truncate it.
        // If a move is fully before print_time, keep it and stop.

        // Retain logic is easier for this:
        // Keep history items that are fully before print_time.
        // For items that straddle, truncate them.
        // Discard items fully after print_time.
        let mut valid_history = Vec::new();
        for mut hist_move in self.history.drain(..) { // Draining here, then rebuilding.
            if hist_move.print_time < print_time {
                if hist_move.print_time + hist_move.move_t > print_time {
                    hist_move.move_t = print_time - hist_move.print_time;
                }
                valid_history.push(hist_move);
                // Since history is traversed from head (oldest non-discarded),
                // once we keep one, all subsequent ones in original history were newer,
                // so this break from C is effectively "stop processing after first one kept/truncated".
                // But our loop iterates all. C adds new marker to head.
                // The C loop means: remove from head of history until you find a move
                // that starts before print_time. Truncate that one if needed. Stop.
                // This means only the *most recent* historical move *prior* to print_time is kept/truncated.
            }
            // Moves starting at or after print_time are discarded by not pushing to valid_history.
        }
        // The C code implies that after this pruning, only one move (the one straddling or immediately before print_time)
        // or no moves would remain before adding the new marker.
        // Let's re-think the pruning. It removes from the *head* of history (which are newer moves).
        // So `list_first_entry` gets the newest item in history.

        // Corrected pruning logic for history:
        // The history is ordered with newest (most recent print_time) at the head (index 0).
        while let Some(mut head_hist_move) = self.history.first_mut() {
            if head_hist_move.print_time < print_time {
                // This move starts before the set_position event.
                if head_hist_move.print_time + head_hist_move.move_t > print_time {
                    // It straddles the event, so truncate it.
                    head_hist_move.move_t = print_time - head_hist_move.print_time;
                }
                // This move (potentially truncated) is the new end of history. Stop pruning.
                break;
            } else {
                // This move is entirely at or after the set_position event. Remove it.
                self.history.remove(0);
            }
        }

        // Add a marker to the trapq history
        // C: struct move *m = move_alloc(); ... list_add_head(&m->node, &tq->history);
        let marker_move = Move {
            print_time,
            move_t: 0.0, // Marker move has zero duration
            start_v: 0.0,
            half_accel: 0.0,
            start_pos: pos,
            axes_r: Coord { x: 0.0, y: 0.0, z: 0.0 }, // No direction/length
        };
        self.history.insert(0, marker_move); // Add to head of history
    }

    /// Return history of movement queue.
    /// Corresponds to C: int trapq_extract_old(...)
    #[cfg(feature = "alloc")]
    pub fn extract_old(
        &self,
        max_moves: usize,
        start_time: f64,
        end_time: f64,
    ) -> Vec<PullMove> {
        let mut result = Vec::new();
        if max_moves == 0 {
            return result;
        }

        for m_hist in &self.history {
            // In C: if (start_time >= m->print_time + m->move_t || res >= max) break;
            // This means moves that *end* at or before start_time are too old.
            if start_time >= m_hist.print_time + m_hist.move_t {
                break; // This move and subsequent (older) ones are too old
            }

            // In C: if (end_time <= m->print_time) continue;
            // This means moves that *start* at or after end_time are too new.
            if end_time <= m_hist.print_time {
                continue; // This move is too new, check the next (older) one
            }

            // If we reach here, the move is within the time window [start_time, end_time)
            // considering move intervals.
            let p_move = PullMove {
                print_time: m_hist.print_time,
                move_t: m_hist.move_t,
                start_v: m_hist.start_v,
                accel: 2.0 * m_hist.half_accel,
                start_pos: m_hist.start_pos,
                axes_r: m_hist.axes_r,
            };
            result.push(p_move);

            if result.len() >= max_moves {
                break; // Reached max number of moves to extract
            }
        }
        result
    }

    /// Returns the print time when the last move in the queue finishes.
    /// This provides similar information to accessing the C version's tail sentinel's print_time.
    #[cfg(feature = "alloc")]
    pub fn last_move_end_time(&self) -> Option<f64> {
        self.moves.last().map(|last_move| last_move.print_time + last_move.move_t)
    }

    /// Returns the position where the last move in the queue finishes.
    /// This provides similar information to accessing the C version's tail sentinel's start_pos.
    #[cfg(feature = "alloc")]
    pub fn last_move_end_pos(&self) -> Option<Coord> {
        self.moves.last().map(|last_move| {
            let move_dist = last_move.get_distance(last_move.move_t);
            Coord {
                x: last_move.start_pos.x + last_move.axes_r.x * move_dist,
                y: last_move.start_pos.y + last_move.axes_r.y * move_dist,
                z: last_move.start_pos.z + last_move.axes_r.z * move_dist,
            }
        })
    }
}


// Vec2D from itersolve.rs (and C trapq.h's struct coord) seems to be a 2D vector.
// The existing `src/itersolve.rs` uses `trapq::Vec2D`.
// `trapq.h` has `struct coord` which is 3D.
// Let's define Vec2D here as it's a common kinematic structure.
// The original `itersolve.rs` might have intended this to be separate.
// For now, placing it here for cohesion with `Coord` (which is 3D).
// If `itersolve.rs` truly needs a distinct 2D type not tied to `trapq.c`'s 3D `coord`,
// we can adjust. `stepper_kinematics.commanded_pos` uses it.
#[derive(Debug, Copy, Clone, PartialEq, Default)]
pub struct Vec2D {
    pub x: f64,
    pub y: f64,
}

impl Vec2D {
    pub fn new(x: f64, y: f64) -> Self {
        Vec2D { x, y }
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use float_cmp::assert_approx_eq;

    // Helper to create a default TrapQ for tests that need alloc
    #[cfg(feature = "alloc")]
    fn new_test_tq() -> TrapQ {
        TrapQ::new()
    }

    #[test]
    fn coord_creation() {
        let c = Coord::new(1.0, 2.0, 3.0);
        assert_eq!(c.x, 1.0);
        assert_eq!(c.y, 2.0);
        assert_eq!(c.z, 3.0);
    }

    #[test]
    fn move_creation() {
        let start_pos = Coord::new(0.0, 0.0, 0.0);
        let axes_r = Coord::new(1.0, 0.0, 0.0); // Moving 1 unit in X
        let m = Move::new(10.0, 1.0, 0.0, 0.5, start_pos, axes_r);
        assert_eq!(m.print_time, 10.0);
        assert_eq!(m.move_t, 1.0);
        assert_eq!(m.start_v, 0.0);
        assert_eq!(m.half_accel, 0.5);
        assert_eq!(m.start_pos, start_pos);
        assert_eq!(m.axes_r, axes_r);
    }

    #[test]
    fn move_get_distance_calculation() {
        let start_pos = Coord::new(0.0, 0.0, 0.0);
        let m = Move::new(0.0, 2.0, 0.0, 1.0, start_pos, Coord::new(1.0,0.0,0.0));
        assert_approx_eq!(f64, m.get_distance(0.0), 0.0);
        assert_approx_eq!(f64, m.get_distance(1.0), 1.0);
        assert_approx_eq!(f64, m.get_distance(2.0), 4.0);
        assert_approx_eq!(f64, m.get_distance(3.0), 4.0); // Clamped
        assert_approx_eq!(f64, m.get_distance(-1.0), 0.0); // Clamped
    }

     #[test]
    fn move_get_coord_calculation() {
        let start_pos = Coord::new(1.0, 2.0, 3.0);
        let unit_axes_r = Coord::new(1.0, 0.0, 0.0);
        let m_unit = Move::new(0.0, 2.0, 0.0, 1.0, start_pos, unit_axes_r);
        assert_eq!(m_unit.get_coord(1.0), Coord::new(2.0, 2.0, 3.0));
        assert_eq!(m_unit.get_coord(2.0), Coord::new(5.0, 2.0, 3.0));
    }

    #[test]
    fn vec2d_creation() {
        let v = Vec2D::new(1.0, 2.0);
        assert_eq!(v.x, 1.0);
        assert_eq!(v.y, 2.0);
    }

    #[cfg(feature = "alloc")]
    #[test]
    fn trapq_creation_with_alloc() {
        let tq = new_test_tq();
        assert!(tq.moves.is_empty());
        assert!(tq.history.is_empty());
    }

    #[cfg(feature = "alloc")]
    mod trapq_append_tests {
        use super::*;

        #[test]
        fn append_single_cruise_move() {
            let mut tq = new_test_tq();
            let start_pos = Coord::new(0.0, 0.0, 0.0);
            let axes_r = Coord::new(1.0, 0.0, 0.0);
            tq.append(0.0, 0.0, 1.0, 0.0, start_pos, axes_r, 0.0, 5.0, 0.0);

            assert_eq!(tq.moves.len(), 1);
            let m = &tq.moves[0];
            assert_approx_eq!(f64, m.print_time, 0.0);
            assert_approx_eq!(f64, m.move_t, 1.0);
            assert_approx_eq!(f64, m.start_v, 5.0); // cruise_v for cruise segment
            assert_approx_eq!(f64, m.half_accel, 0.0);
            assert_eq!(m.start_pos, start_pos);
            assert_eq!(m.axes_r, axes_r);
        }

        #[test]
        fn append_full_accel_cruise_decel_move() {
            let mut tq = new_test_tq();
            let initial_pos = Coord::new(0.0, 0.0, 0.0);
            let axes_r = Coord::new(1.0, 0.0, 0.0); // Move 1 unit along X effectively
            let accel = 10.0;
            let start_v = 0.0;
            let cruise_v = 5.0;
            // accel_t = (cruise_v - start_v) / accel = (5-0)/10 = 0.5
            // decel_t = cruise_v / accel = 5/10 = 0.5
            let accel_t = 0.5;
            let cruise_t = 1.0;
            let decel_t = 0.5;

            tq.append(0.0, accel_t, cruise_t, decel_t, initial_pos, axes_r, start_v, cruise_v, accel);
            assert_eq!(tq.moves.len(), 3);

            // Accel phase
            let m_accel = &tq.moves[0];
            assert_approx_eq!(f64, m_accel.print_time, 0.0);
            assert_approx_eq!(f64, m_accel.move_t, accel_t);
            assert_approx_eq!(f64, m_accel.start_v, start_v);
            assert_approx_eq!(f64, m_accel.half_accel, 0.5 * accel);
            assert_eq!(m_accel.start_pos, initial_pos);

            // Expected end position of accel phase
            let accel_dist = (start_v + 0.5 * accel * accel_t) * accel_t;
            let cruise_start_pos = Coord {
                x: initial_pos.x + axes_r.x * accel_dist,
                y: initial_pos.y + axes_r.y * accel_dist,
                z: initial_pos.z + axes_r.z * accel_dist,
            };

            // Cruise phase
            let m_cruise = &tq.moves[1];
            assert_approx_eq!(f64, m_cruise.print_time, accel_t);
            assert_approx_eq!(f64, m_cruise.move_t, cruise_t);
            assert_approx_eq!(f64, m_cruise.start_v, cruise_v);
            assert_approx_eq!(f64, m_cruise.half_accel, 0.0);
            assert_eq!(m_cruise.start_pos, cruise_start_pos);

            // Expected end position of cruise phase
            let cruise_dist = cruise_v * cruise_t;
            let decel_start_pos = Coord {
                x: cruise_start_pos.x + axes_r.x * cruise_dist,
                y: cruise_start_pos.y + axes_r.y * cruise_dist,
                z: cruise_start_pos.z + axes_r.z * cruise_dist,
            };

            // Decel phase
            let m_decel = &tq.moves[2];
            assert_approx_eq!(f64, m_decel.print_time, accel_t + cruise_t);
            assert_approx_eq!(f64, m_decel.move_t, decel_t);
            assert_approx_eq!(f64, m_decel.start_v, cruise_v);
            assert_approx_eq!(f64, m_decel.half_accel, -0.5 * accel);
            assert_eq!(m_decel.start_pos, decel_start_pos);
        }

        #[test]
        fn add_move_segment_no_gap() {
            let mut tq = new_test_tq();
            let m1 = Move::new(0.0, 1.0, 0.0, 0.0, Coord::default(), Coord::default());
            tq.add_move_segment(m1.clone());

            let m2 = Move::new(1.0, 1.0, 0.0, 0.0, Coord::default(), Coord::default());
            tq.add_move_segment(m2.clone());

            assert_eq!(tq.moves.len(), 2);
            assert_eq!(tq.moves[0], m1);
            assert_eq!(tq.moves[1], m2);
        }

        #[test]
        fn add_move_segment_with_gap_inserts_null_move() {
            let mut tq = new_test_tq();
            let m1_pos = Coord::new(1.0,1.0,1.0);
            let m1 = Move::new(0.0, 1.0, 5.0, 0.5, m1_pos, Coord::new(1.0,0.0,0.0));
            tq.add_move_segment(m1.clone());

            // m2 starts at time 3.0, m1 ends at 1.0. Gap of 2.0.
            let m2_pos = Coord::new(10.0,10.0,10.0);
            let m2 = Move::new(3.0, 1.0, 0.0, 0.0, m2_pos, Coord::new(0.0,1.0,0.0));
            tq.add_move_segment(m2.clone());

            assert_eq!(tq.moves.len(), 3);
            assert_eq!(tq.moves[0], m1);

            let null_move = &tq.moves[1];
            assert_approx_eq!(f64, null_move.print_time, 1.0); // Starts after m1 ends
            assert_approx_eq!(f64, null_move.move_t, 2.0);   // Fills the gap (3.0 - 1.0)
            assert_approx_eq!(f64, null_move.start_v, 0.0);
            assert_approx_eq!(f64, null_move.half_accel, 0.0);
            assert_eq!(null_move.start_pos, m2_pos); // Null move takes start_pos of the gapped move
            assert_eq!(null_move.axes_r, Coord::new(0.0,0.0,0.0)); // No displacement

            assert_eq!(tq.moves[2], m2);
        }

        #[test]
        fn add_move_segment_first_move_large_print_time() {
            let mut tq = new_test_tq();
            // First move starts at time 5.0, MAX_NULL_MOVE_DURATION is 1.0
            // Expect a null move from 0.0 for (5.0 - 1.0) = 4.0 seconds
            let m1_pos = Coord::new(1.0,0.0,0.0);
            let m1 = Move::new(5.0, 1.0, 0.0, 0.0, m1_pos, Coord::new(1.0,0.0,0.0));
            tq.add_move_segment(m1.clone());

            assert_eq!(tq.moves.len(), 2);

            let null_move = &tq.moves[0];
            assert_approx_eq!(f64, null_move.print_time, 0.0);
            assert_approx_eq!(f64, null_move.move_t, 4.0); // 5.0 - MAX_NULL_MOVE_DURATION (1.0)
            assert_approx_eq!(f64, null_move.start_v, 0.0);
            assert_approx_eq!(f64, null_move.half_accel, 0.0);
            assert_eq!(null_move.start_pos, m1_pos);
            assert_eq!(null_move.axes_r, Coord::new(0.0,0.0,0.0));

            assert_eq!(tq.moves[1], m1);
        }
         #[test]
        fn add_move_segment_first_move_small_print_time() {
            let mut tq = new_test_tq();
            // First move starts at time 0.5, less than MAX_NULL_MOVE_DURATION (1.0)
            // No leading null move expected.
            let m1 = Move::new(0.5, 1.0, 0.0, 0.0, Coord::default(), Coord::default());
            tq.add_move_segment(m1.clone());

            assert_eq!(tq.moves.len(), 1);
            assert_eq!(tq.moves[0], m1);
        }

        #[test]
        fn add_move_segment_prev_move_is_heuristic_initial_state_large_gap() {
            let mut tq = new_test_tq();
            // Heuristic initial state: print_time=0, move_t=0
            let initial_marker = Move::new(0.0, 0.0, 0.0, 0.0, Coord::new(0.,0.,0.), Coord::new(0.,0.,0.));
            tq.moves.push(initial_marker); // Manually add a "zero" move

            // New move starts at 5.0. MAX_NULL_MOVE_DURATION is 1.0
            // prev_move.print_time (0.0) + prev_move.move_t (0.0) < m.print_time (5.0) -> gap
            // prev_move.print_time == 0.0 && prev_move.move_t == 0.0 -> true
            // m.print_time (5.0) > MAX_NULL_MOVE_DURATION (1.0) -> true
            // So, null_move_print_time = m.print_time - MAX_NULL_MOVE_DURATION = 5.0 - 1.0 = 4.0
            // null_move.move_t = m.print_time - null_move_print_time = 5.0 - 4.0 = 1.0
            let m_actual_pos = Coord::new(1.0,0.0,0.0);
            let m_actual = Move::new(5.0, 1.0, 0.0, 0.0, m_actual_pos, Coord::new(1.0,0.0,0.0));
            tq.add_move_segment(m_actual.clone());

            assert_eq!(tq.moves.len(), 3); // initial_marker, null_move, m_actual

            let null_move = &tq.moves[1];
            assert_approx_eq!(f64, null_move.print_time, 4.0);
            assert_approx_eq!(f64, null_move.move_t, 1.0);
            assert_eq!(null_move.start_pos, m_actual_pos);
        }
    }

    // Test for no_alloc would require defining MAX_MOVES etc.
    // #[test]
    // fn trapq_creation_no_alloc() {
    //     let tq = TrapQ::new_no_alloc();
    //     // Add assertions based on fixed-size array initialization
    // }

    #[cfg(feature = "alloc")]
    mod trapq_extract_old_tests {
        use super::*; // Imports Move, Coord, TrapQ, PullMove, new_test_tq, etc.
        use float_cmp::assert_approx_eq;

        // Helper to create a non-null move for history
        fn history_move(print_time: f64, move_t: f64, id: f64) -> Move {
            Move::new(
                print_time,
                move_t,
                1.0, // start_v
                0.5, // half_accel (implies accel = 1.0 for PullMove)
                Coord::new(id, id, id), // start_pos
                Coord::new(1.0, 0.0, 0.0), // axes_r
            )
        }

        #[test]
        fn extract_old_empty_history() {
            let tq = new_test_tq();
            let result = tq.extract_old(5, 0.0, 10.0);
            assert!(result.is_empty());
        }

        #[test]
        fn extract_old_max_moves_zero() {
            let mut tq = new_test_tq();
            tq.history.push(history_move(0.0, 1.0, 1.0));
            let result = tq.extract_old(0, 0.0, 10.0);
            assert!(result.is_empty());
        }

        #[test]
        fn extract_old_no_moves_in_time_window() {
            let mut tq = new_test_tq();
            // History (newest to oldest): m(2.0, 1.0), m(0.0, 1.0)
            tq.history.push(history_move(0.0, 1.0, 1.0)); // Ends at 1.0
            tq.history.push(history_move(2.0, 1.0, 2.0)); // Ends at 3.0 (This is at index 0)

            // Window [1.0, 2.0)
            // m(2.0, 1.0) starts at 2.0, so end_time(2.0) <= m.print_time(2.0) -> continue (skip)
            // m(0.0, 1.0) ends at 1.0, so start_time(1.0) >= m.print_time(0.0)+m.move_t(1.0) -> break
            let result = tq.extract_old(5, 1.0, 2.0);
            assert!(result.is_empty());
        }

        #[test]
        fn extract_old_all_moves_fit_max_moves_and_window() {
            let mut tq = new_test_tq();
            let m1 = history_move(0.0, 1.0, 1.0); // Ends 1.0
            let m2 = history_move(1.0, 1.0, 2.0); // Ends 2.0
            // History (newest to oldest): m2, m1
            tq.history.push(m1.clone());
            tq.history.push(m2.clone());


            let result = tq.extract_old(5, 0.0, 2.5);
            assert_eq!(result.len(), 2);
            // Order in result should be same as history: newest to oldest
            assert_approx_eq!(f64, result[0].print_time, m2.print_time);
            assert_approx_eq!(f64, result[0].accel, 2.0 * m2.half_accel);
            assert_eq!(result[0].start_pos, m2.start_pos);

            assert_approx_eq!(f64, result[1].print_time, m1.print_time);
            assert_approx_eq!(f64, result[1].accel, 2.0 * m1.half_accel);
            assert_eq!(result[1].start_pos, m1.start_pos);
        }

        #[test]
        fn extract_old_limited_by_max_moves() {
            let mut tq = new_test_tq();
            let m1 = history_move(0.0, 1.0, 1.0);
            let m2 = history_move(1.0, 1.0, 2.0);
            let m3 = history_move(2.0, 1.0, 3.0);
            // History (newest to oldest): m3, m2, m1
            tq.history.push(m1.clone());
            tq.history.push(m2.clone());
            tq.history.push(m3.clone());

            let result = tq.extract_old(2, 0.0, 3.5); // Max 2 moves
            assert_eq!(result.len(), 2);
            assert_approx_eq!(f64, result[0].print_time, m3.print_time);
            assert_approx_eq!(f64, result[1].print_time, m2.print_time);
        }

        #[test]
        fn extract_old_limited_by_start_time() {
            let mut tq = new_test_tq();
            let m1 = history_move(0.0, 1.0, 1.0); // Ends 1.0
            let m2 = history_move(1.0, 1.0, 2.0); // Ends 2.0
            let m3 = history_move(2.0, 1.0, 3.0); // Ends 3.0
            // History: m3, m2, m1
            tq.history.push(m1.clone());
            tq.history.push(m2.clone());
            tq.history.push(m3.clone());

            // start_time = 1.5.
            // m3 (pt=2, mt=1, ends=3): 1.5 < 3.0 (ok). end_time(3.5) > 2.0 (ok). Include.
            // m2 (pt=1, mt=1, ends=2): 1.5 < 2.0 (ok). end_time(3.5) > 1.0 (ok). Include.
            // m1 (pt=0, mt=1, ends=1): 1.5 >= 1.0 (start_time >= move_end_time). Break.
            let result = tq.extract_old(5, 1.5, 3.5);
            assert_eq!(result.len(), 2);
            assert_approx_eq!(f64, result[0].print_time, m3.print_time);
            assert_approx_eq!(f64, result[1].print_time, m2.print_time);
        }

        #[test]
        fn extract_old_limited_by_end_time() {
            let mut tq = new_test_tq();
            let m1 = history_move(0.0, 1.0, 1.0); // Ends 1.0
            let m2 = history_move(1.0, 1.0, 2.0); // Ends 2.0
            let m3 = history_move(2.0, 1.0, 3.0); // Ends 3.0
            // History: m3, m2, m1
            tq.history.push(m1.clone());
            tq.history.push(m2.clone());
            tq.history.push(m3.clone());

            // end_time = 1.5
            // m3 (pt=2, mt=1): end_time(1.5) <= pt(2.0). Skip.
            // m2 (pt=1, mt=1): end_time(1.5) > pt(1.0) (ok). start_time(0.0) < ends(2.0) (ok). Include.
            // m1 (pt=0, mt=1): end_time(1.5) > pt(0.0) (ok). start_time(0.0) < ends(1.0) (ok). Include.
            let result = tq.extract_old(5, 0.0, 1.5);
            assert_eq!(result.len(), 2);
            assert_approx_eq!(f64, result[0].print_time, m2.print_time); // m3 skipped
            assert_approx_eq!(f64, result[1].print_time, m1.print_time);
        }

        #[test]
        fn extract_old_exact_time_match() {
            let mut tq = new_test_tq();
            let m = history_move(1.0, 1.0, 1.0); // pt=1, ends=2
            tq.history.push(m.clone());

            // Window [1.0, 2.0) should include the move
            // start_time(1.0) < m.print_time(1.0)+m.move_t(1.0) -> 1.0 < 2.0 (ok)
            // end_time(2.0) > m.print_time(1.0) -> 2.0 > 1.0 (ok)
            let result = tq.extract_old(5, 1.0, 2.0);
            assert_eq!(result.len(), 1);
            assert_approx_eq!(f64, result[0].print_time, m.print_time);

            // Window [1.0, 1.9) should include the move
            let result_tight_end = tq.extract_old(5, 1.0, 1.9);
            assert_eq!(result_tight_end.len(), 1);

            // Window [1.1, 2.0) should include the move
            let result_tight_start = tq.extract_old(5, 1.1, 2.0);
            assert_eq!(result_tight_start.len(), 1);

            // Window covering exactly the move's print_time to print_time + move_t
            // Move starts at 1.0, ends at 2.0.
            // start_time = 1.0. Condition: 1.0 < 1.0 + 1.0 (true)
            // end_time = 2.0. Condition: 2.0 <= 1.0 (false) -> so it is included.
            let result_exact = tq.extract_old(1, m.print_time, m.print_time + m.move_t);
            assert_eq!(result_exact.len(), 1);


            // Move starts at 1.0, ends at 2.0
            // Test case: start_time = 2.0 (exactly when move ends)
            // Condition: start_time >= m.print_time + m.move_t  => 2.0 >= 1.0 + 1.0  => 2.0 >= 2.0 (true) -> break. So, not included.
            let result_start_at_end = tq.extract_old(1, m.print_time + m.move_t, m.print_time + m.move_t + 1.0);
            assert!(result_start_at_end.is_empty());

            // Test case: end_time = 1.0 (exactly when move starts)
            // Condition: end_time <= m.print_time => 1.0 <= 1.0 (true) -> continue. So, not included.
            let result_end_at_start = tq.extract_old(1, m.print_time - 1.0, m.print_time);
            assert!(result_end_at_start.is_empty());
        }
    }

    #[cfg(feature = "alloc")]
    mod trapq_last_move_info_tests {
        use super::*;
        use float_cmp::assert_approx_eq;

        fn create_move(print_time: f64, move_t: f64, start_pos: Coord, axes_r: Coord) -> Move {
            Move::new(print_time, move_t, 1.0, 0.5, start_pos, axes_r)
        }

        #[test]
        fn last_move_info_empty_queue() {
            let tq = new_test_tq();
            assert!(tq.last_move_end_time().is_none());
            assert!(tq.last_move_end_pos().is_none());
        }

        #[test]
        fn last_move_info_single_move() {
            let mut tq = new_test_tq();
            let start_pos = Coord::new(1.0, 2.0, 3.0);
            let axes_r = Coord::new(1.0, 0.0, 0.0); // Moves in X
            let m = create_move(10.0, 2.0, start_pos, axes_r); // pt=10, mt=2
                                                                 // start_v=1, half_accel=0.5
            tq.moves.push(m.clone());

            let expected_end_time = 10.0 + 2.0;
            assert!(tq.last_move_end_time().is_some());
            assert_approx_eq!(f64, tq.last_move_end_time().unwrap(), expected_end_time);

            // dist = start_v * t + half_accel * t^2 = 1*2 + 0.5*2^2 = 2 + 2 = 4
            let expected_dist = m.start_v * m.move_t + m.half_accel * m.move_t * m.move_t;
            let expected_end_pos = Coord {
                x: start_pos.x + axes_r.x * expected_dist, // 1 + 1*4 = 5
                y: start_pos.y + axes_r.y * expected_dist, // 2 + 0*4 = 2
                z: start_pos.z + axes_r.z * expected_dist, // 3 + 0*4 = 3
            };
            assert!(tq.last_move_end_pos().is_some());
            let actual_end_pos = tq.last_move_end_pos().unwrap();
            assert_approx_eq!(f64, actual_end_pos.x, expected_end_pos.x);
            assert_approx_eq!(f64, actual_end_pos.y, expected_end_pos.y);
            assert_approx_eq!(f64, actual_end_pos.z, expected_end_pos.z);
        }

        #[test]
        fn last_move_info_multiple_moves() {
            let mut tq = new_test_tq();
            let start_pos1 = Coord::new(0.0, 0.0, 0.0);
            let axes_r1 = Coord::new(1.0, 0.0, 0.0);
            let m1 = create_move(0.0, 1.0, start_pos1, axes_r1); // Ends at t=1.0, pos_x=1.5
            tq.moves.push(m1.clone());

            let start_pos2 = Coord::new(1.5, 0.0, 0.0); // Start where m1 ended
            let axes_r2 = Coord::new(0.0, 1.0, 0.0); // Moves in Y
            let m2 = create_move(1.0, 2.0, start_pos2, axes_r2); // pt=1, mt=2. start_v=1, half_accel=0.5
            tq.moves.push(m2.clone()); // This is the last move

            let expected_end_time = 1.0 + 2.0; // m2.print_time + m2.move_t
            assert!(tq.last_move_end_time().is_some());
            assert_approx_eq!(f64, tq.last_move_end_time().unwrap(), expected_end_time);

            // For m2: dist = 1*2 + 0.5*2^2 = 2 + 2 = 4
            let expected_dist_m2 = m2.start_v * m2.move_t + m2.half_accel * m2.move_t * m2.move_t;
            let expected_end_pos_m2 = Coord {
                x: start_pos2.x + axes_r2.x * expected_dist_m2, // 1.5 + 0*4 = 1.5
                y: start_pos2.y + axes_r2.y * expected_dist_m2, // 0.0 + 1*4 = 4.0
                z: start_pos2.z + axes_r2.z * expected_dist_m2, // 0.0 + 0*4 = 0.0
            };
            assert!(tq.last_move_end_pos().is_some());
            let actual_end_pos = tq.last_move_end_pos().unwrap();
            assert_approx_eq!(f64, actual_end_pos.x, expected_end_pos_m2.x);
            assert_approx_eq!(f64, actual_end_pos.y, expected_end_pos_m2.y);
            assert_approx_eq!(f64, actual_end_pos.z, expected_end_pos_m2.z);
        }
         #[test]
        fn last_move_info_move_with_zero_duration() {
            let mut tq = new_test_tq();
            let start_pos = Coord::new(1.0, 1.0, 1.0);
            let m = create_move(5.0, 0.0, start_pos, Coord::new(1.0,0.0,0.0)); // move_t = 0
            tq.moves.push(m.clone());

            let expected_end_time = 5.0 + 0.0;
            assert_approx_eq!(f64, tq.last_move_end_time().unwrap(), expected_end_time);

            // dist for zero duration move is 0
            let expected_end_pos = start_pos; // Position should not change
            let actual_end_pos = tq.last_move_end_pos().unwrap();
            assert_approx_eq!(f64, actual_end_pos.x, expected_end_pos.x);
            assert_approx_eq!(f64, actual_end_pos.y, expected_end_pos.y);
            assert_approx_eq!(f64, actual_end_pos.z, expected_end_pos.z);
        }
    }
}
