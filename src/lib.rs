//! Code for using Rust in FTC robot code.

use std::fmt::Display;

#[cfg(feature = "proc-macro")]
pub use ftc_rust_proc::ftc;
pub use jni;
use jni::{objects::JObject, refs::Global, strings::JNIString, vm::JavaVM};

use crate::hardware::Hardware;

pub mod hardware;

#[macro_use]
mod macros;

/// A wrapper for accessing telemetry-related methods.
#[derive(Debug)]
pub struct Telemetry {
    /// The environment.
    vm: JavaVM,
    /// The actual telemetry object. Should be org/firstinspires/ftc/robotcore/external/Telemetry.
    telemetry: Global<JObject<'static>>,
}

impl Telemetry {
    /// Adds an item to the end if the telemetry being built for driver station display. The caption
    /// and value are shown on the driver station separated by the caption value separator. The
    /// item is removed if clear or `clear_all` is called.
    pub fn add_data(&self, caption: impl Display, value: impl Display) {
        self.vm
            .attach_current_thread(|env| {
                let caption = new_string!(env env, caption.to_string()).unwrap();
                let value = new_string!(env env, value.to_string()).unwrap();
                call_method!(
                    env env,
                    self.telemetry,
                    "addData",
                    "(Ljava/lang/String;Ljava/lang/Object;)V",
                    [&caption, &value]
                )
                .unwrap();
                jni::errors::Result::Ok(()) // cannot return a reference
            })
            .unwrap();
    }
    /// Sends the receiver Telemetry to the driver station if more than the transmission interval
    /// has elapsed since the last transmission, or schedules the transmissionof the receiver
    /// should no subsequent Telemetry state be scheduled for transmission beforethe
    /// transmission interval expires.
    pub fn update(&self) {
        call_method!(
            void self,
            self.telemetry,
            "update",
            "()V",
            []
        );
    }
    /// Removes all items from the receiver whose value is not to be retained.
    pub fn clear(&self) {
        call_method!(
            void self,
            self.telemetry,
            "clear",
            "()V",
            []
        );
    }
    /// Removes all items, lines, and actions from the receiver
    pub fn clear_all(&self) {
        call_method!(
            void self,
            self.telemetry,
            "clearAll",
            "()V",
            []
        );
    }
}

/// A context used for accessing the Java runtime.
#[derive(Debug)]
pub struct FtcContext {
    /// The java environment.
    vm: JavaVM,
    /// The op mode class.
    this: Global<JObject<'static>>,
}

impl<'a, 'local> FtcContext {
    /// Create a new context.
    pub fn new(env: &'a mut jni::Env<'local>, this: JObject<'local>) -> Self {
        Self {
            this: env.new_global_ref(this).unwrap(),
            vm: env.get_java_vm(),
        }
    }
    /// Return a telemetry struct.
    pub fn telemetry(&mut self) -> Telemetry {
        let telemetry = self
            .vm
            .attach_current_thread(|env| {
                new_global!(
                    env,
                    env.get_field(
                        &self.this,
                        JNIString::new("telemetry"),
                        JNIString::new("Lorg/firstinspires/ftc/robotcore/external/Telemetry;"),
                    )
                    .unwrap()
                    .l()
                    .unwrap()
                )
            })
            .unwrap();

        Telemetry {
            vm: self.vm.clone(),
            telemetry,
        }
    }
    /// Return a hardware struct.
    pub fn hardware(&mut self) -> Hardware {
        let hardware_map = self
            .vm
            .attach_current_thread(|env| {
                new_global!(
                    env,
                    env.get_field(
                        &self.this,
                        JNIString::new("hardwareMap"),
                        JNIString::new("Lcom/qualcomm/robotcore/hardware/HardwareMap;"),
                    )
                    .unwrap()
                    .l()
                    .unwrap()
                )
            })
            .unwrap();

        Hardware {
            vm: self.vm.clone(),
            hardware_map,
        }
    }
    /// Wait for the driver to press play.
    pub fn wait_for_start(&mut self) {
        call_method!(void self, self.this, "waitForStart", "()V", []);
    }
    /// Sleeps for the given amount of milliseconds, or until the thread is interrupted (which
    /// usually indicates that the `OpMode` has been stopped).
    pub fn sleep_ms(&mut self, time: i64) {
        call_method!(void self, self.this, "sleep", "(J)V", [time]);
    }
    /// Sleeps for the given number of seconds.
    pub fn sleep_s(&mut self, time: f64) {
        self.sleep_ms((time * 1000.0) as i64);
    }
}
