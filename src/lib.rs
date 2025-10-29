//! Rust Project

use core::ops::Add;

/// Add two values together.
///
/// # Errors
/// This is an example that does not error.
pub fn add<T: Add>(a: T, b: T) -> Result<T::Output, Box<dyn std::error::Error>> {
    Ok(a + b)
}
