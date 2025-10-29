//! Code for using Rust in FTC robot code.

#[cfg(feature = "proc-macro")]
pub use ftc_rust_proc::ftc;

/// A direction for motors.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum Direction {
    /// Turn forward. Commonly clockwise.
    #[default]
    Forward,
    /// Turn backward. Commonly counterclockwise.
    Reverse
}