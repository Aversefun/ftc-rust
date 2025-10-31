//! Implementations of [`hardware::Device`](crate::hardware::Device).

use jni::{objects::JObject, refs::Global, JavaVM};

use crate::{call_method, hardware::{Direction, IntoJniObject as _, RunMode, ZeroPowerBehavior}};

/// Easily define a basic device.
macro_rules! device {
    ($(#[$attr:meta])* $name:ident, JAVA_CLASS = $java_class:literal ; JNI_CLASS = $jni_class:literal $(;)?) => {
        $(#[$attr])*
        pub struct $name {
            /// The environment.
            vm: JavaVM,
            /// The actual object.
            object: Global<JObject<'static>>,
        }

        impl $crate::hardware::Device for $name {
            const JAVA_CLASS: &'static str = $java_class;
            const JNI_CLASS: &'static str = $jni_class;
            fn from_java(vm: JavaVM, object: Global<JObject<'static>>) -> Self {
                Self {
                    vm,
                    object,
                }
            }
        }
    };
}

device!(
    /// `DcMotor` provides access to full-featured motor functionality.
    DcMotor,
    JAVA_CLASS = "com.qualcomm.robotcore.hardware.DcMotor";
    JNI_CLASS = "com/qualcomm/robotcore/hardware/DcMotor";
);

impl DcMotor {
    /// Sets the logical direction in which this motor operates.
    pub fn set_direction(&self, dir: Direction) {
        self.vm.attach_current_thread(|env| {
            let obj = dir.into_jni_object(env);
            call_method!(
                env env,
                self.object,
                "setDirection",
                format!("(L{};)V", Direction::JNI_CLASS),
                [&obj]
            ).unwrap();
            jni::errors::Result::Ok(()) // cannot return a reference
        }).unwrap();
    }

    /// Returns the current logical direction in which this motor is set as operating.
    pub fn get_direction(&self) -> Direction {
        let res = call_method!(
            obj self,
            self.object,
            "getDirection",
            format!("()L{};", Direction::JNI_CLASS),
            []
        );
        Direction::from_jni_object(self.vm.clone(), &res)
    }

    /// Sets the power level of the motor, expressed as a fraction of the maximum possible power / speed supported
    /// according to the run mode in which the motor is operating.
    ///
    /// Setting a power level of zero will brake the motor
    pub fn set_power(&self, power: f64) {
        call_method!(void self, self.object, "setPower", "(D)V", [power]);
    }

    /// Sets the power level of the motor, expressed as a fraction of the maximum possible power / speed supported
    /// according to the run mode in which the motor is operating.
    ///
    /// Setting a power level of zero will brake the motor
    #[must_use]
    pub fn get_power(&self) -> f64 {
        call_method!(double self, self.object, "getPower", "()D",
            [])
    }

    /// Sets the behavior of the motor when a power level of zero is applied.
    pub fn set_zero_power_behavior(&self, zpb: ZeroPowerBehavior) {
        self.vm.attach_current_thread(|env| {
            let obj = zpb.into_jni_object(env);
            call_method!(
                env env,
                self.object,
                "setZeroPowerBehavior",
                format!("(L{};)V", ZeroPowerBehavior::JNI_CLASS),
                [&obj]
            ).unwrap();
            jni::errors::Result::Ok(()) // cannot return a reference
        }).unwrap();
    }

    /// Returns the current behavior of the motor were a power level of zero to be applied.
    pub fn get_zero_power_behavior(&self) -> ZeroPowerBehavior {
        let res = call_method!(
            obj self,
            self.object,
            "getZeroPowerBehavior",
            format!("()L{};", ZeroPowerBehavior::JNI_CLASS),
            []
        );
        ZeroPowerBehavior::from_jni_object(self.vm.clone(), &res)
    }

    /// Sets the desired encoder target position to which the motor should advance or retreat and then actively hold there
    /// at. This behavior is similar to the operation of a servo. The maximum speed at which this advance or retreat occurs
    /// is governed by the power level currently set on the motor. While the motor is advancing or retreating to the desired
    /// target position, `is_busy` will return true.
    /// 
    /// Note that adjustment to a target position is only effective when the motor is in `RunToPosition` `RunMode`. Note
    /// further that, clearly, the motor must be equipped with an encoder in orderfor this mode to function properly.
    pub fn set_target_position(&self, target_pos: i32) {
        call_method!(void self, self.object, "setTargetPosition", "(I)V", [target_pos]);
    }

    /// Returns the current target encoder position for this motor.
    #[must_use]
    pub fn get_target_position(&self) -> i32 {
        call_method!(int self, self.object, "getTargetPosition", "()I",
            [])
    }

    /// Returns true if the motor is currently advancing or retreating to a target position.
    #[must_use]
    pub fn is_busy(&self) -> bool {
        call_method!(bool self, self.object, "isBusy", "()Z",
            [])
    }

    /// Returns the current reading of the encoder for this motor. The units for this reading,
    /// that is, the number of ticks per revolution, are specific to the motor/encoder in question,
    /// and thus are not specified here.
    #[must_use]
    pub fn get_current_position(&self) -> i32 {
        call_method!(int self, self.object, "getCurrentPosition", "()I",
            [])
    }

    /// Sets the behavior of the motor when a power level of zero is applied.
    pub fn set_mode(&self, mode: RunMode) {
        self.vm.attach_current_thread(|env| {
            let obj = mode.into_jni_object(env);
            call_method!(
                env env,
                self.object,
                "setMode",
                format!("(L{};)V", RunMode::JNI_CLASS),
                [&obj]
            ).unwrap();
            jni::errors::Result::Ok(()) // cannot return a reference
        }).unwrap();
    }

    /// Returns the current behavior of the motor were a power level of zero to be applied.
    pub fn get_mode(&self) -> RunMode {
        let res = call_method!(
            obj self,
            self.object,
            "getMode",
            format!("()L{};", RunMode::JNI_CLASS),
            []
        );
        RunMode::from_jni_object(self.vm.clone(), &res)
    }
}
