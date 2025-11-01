//! Code for using Rust in FTC robot code.

use std::{fmt::Display, rc::Rc, sync::RwLock};

#[cfg(feature = "proc-macro")]
pub use ftc_rust_proc::ftc;
pub use jni;
use jni::{
    objects::JObject,
    strings::JNIString, vm::JavaVM,
};

use crate::hardware::Hardware;

pub mod hardware;

#[macro_use]
mod macros;

/// A wrapper for accessing telemetry-related methods.
#[derive(Debug)]
pub struct Telemetry<'a, 'local> {
    /// The environment.
    vm: JavaVM,
    /// The actual telemetry object. Should be org/firstinspires/ftc/robotcore/external/Telemetry.
    telemetry: JObject<'local>,
}

impl Telemetry<'_, '_> {
    /// Adds an item to the end if the telemetry being built for driver station display. The caption and value are shown
    /// on the driver station separated by the caption value separator. The item is removed if clear or `clear_all` is
    /// called.
    pub fn add_data(&self, caption: impl Display, value: impl Display) {
        let caption = self
            .env
            .write()
            .unwrap()
            .new_string(JNIString::new(caption.to_string()))
            .unwrap();
        let value = self
            .env
            .write()
            .unwrap()
            .new_string(JNIString::new(value.to_string()))
            .unwrap();

        call_method!(
            self.vm,
            self.telemetry,
            "addData",
            "(Ljava/lang/String;Ljava/lang/Object;)V",
            [&caption, &value]
        );
    }
    /// Sends the receiver Telemetry to the driver station if more than the transmission interval has elapsed since the last
    /// transmission, or schedules the transmissionof the receiver should no subsequent Telemetry state be scheduled for
    /// transmission beforethe transmission interval expires.
    pub fn update(&self) {
        call_method!(
            self.vm,
            self.telemetry,
            "update",
            "()V",
            []
        );
    }
    /// Removes all items from the receiver whose value is not to be retained.
    pub fn clear(&self) {
        call_method!(
            self.vm,
            self.telemetry,
            "clear",
            "()V",
            []
        );
    }
    /// Removes all items, lines, and actions from the receiver
    pub fn clear_all(&self) {
        call_method!(
            self.vm,
            self.telemetry,
            "clearAll",
            "()V",
            []
        );
    }
}

/// A context used for accessing the Java runtime.
#[derive(Debug)]
pub struct FtcContext<'a, 'local> {
    /// The java environment.
    vm: JavaVM,
    /// The op mode class.
    this: JObject<'local>,
}

impl<'a, 'local> FtcContext<'a, 'local> {
    /// Create a new context.
    pub fn new(env: &'a mut jni::Env<'local>, this: JObject<'local>) -> Self {
        Self {
            vm: env.get_java_vm(),
            this
        }
    }
    /// Return a telemetry struct.
    pub fn telemetry(&mut self) -> Telemetry<'a, 'local> {
        let telemetry = self
            .env
            .write()
            .unwrap()
            .get_field(
                &self.this,
                JNIString::new("telemetry"),
                JNIString::new("Lorg/firstinspires/ftc/robotcore/external/Telemetry;"),
            )
            .unwrap()
            .l()
            .unwrap();

        Telemetry {
            vm: self.vm.clone(),
            telemetry,
        }
    }
    /// Return a hardware struct.
    pub fn hardware(&mut self) -> Hardware<'a, 'local> {
        let hardware_map = self
            .env
            .write()
            .unwrap()
            .get_field(
                &self.this,
                JNIString::new("hardwareMap"),
                JNIString::new("Lcom/qualcomm/robotcore/hardware/HardwareMap;"),
            )
            .unwrap()
            .l()
            .unwrap();

        Hardware {
            env: self.env.clone(),
            hardware_map,
        }
    }
    /// Wait for the driver to press play.
    pub fn wait_for_start(&mut self) {
        call_method!(self.vm, self.this, "waitForStart", "()V", []);
    }
    /// Sleeps for the given amount of milliseconds, or until the thread is interrupted (which usually indicates that the
    /// `OpMode` has been stopped).
    pub fn sleep_ms(&mut self, time: i64) {
        call_method!(self.vm, self.this, "sleep", "(J)V", [time]);
    }
    /// Sleeps for the given number of seconds.
    pub fn sleep_s(&mut self, time: f64) {
        self.sleep_ms((time * 1000.0) as i64);
    }
}
