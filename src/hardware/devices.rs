//! Implementations of [`hardware::Device`](crate::hardware::Device).

use std::{rc::Rc, sync::RwLock};

use jni::objects::JObject;

use crate::{call_method, hardware::{Direction, ZeroPowerBehavior}};

/// Easily define a basic device.
macro_rules! device {
    ($(#[$attr:meta])* $name:ident, JAVA_CLASS = $java_class:literal ; JNI_CLASS = $jni_class:literal $(;)?) => {
        $(#[$attr])*
        pub struct $name<'a, 'local> {
            /// The environment.
            env: Rc<RwLock<&'a mut jni::Env<'local>>>,
            /// The actual object.
            object: JObject<'local>,
        }

        impl<'a, 'local> $crate::hardware::Device<'a, 'local> for $name<'a, 'local> {
            const JAVA_CLASS: &'static str = $java_class;
            const JNI_CLASS: &'static str = $jni_class;
            fn from_java(env: Rc<RwLock<&'a mut jni::Env<'local>>>, object: JObject<'local>) -> Self {
                Self {
                    env,
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

impl DcMotor<'_, '_> {
    /// Sets the logical direction in which this motor operates.
    pub fn set_direction(&self, dir: Direction) {
        let obj = dir.into_jni_object(&self.env);
        call_method!(
            self,
            self.object,
            "setDirection",
            format!("(L{};)V", Direction::JNI_CLASS),
            [&obj]
        );
    }

    /// Returns the current logical direction in which this motor is set as operating.
    pub fn get_direction(&self) -> Direction {
        let res = call_method!(
            self,
            self.object,
            "getDirection",
            format!("()L{};", Direction::JNI_CLASS)
        )
        .l()
        .unwrap();
        Direction::from_jni_object(&self.env, &res)
    }

    /// Sets the power level of the motor, expressed as a fraction of the maximum possible power / speed supported
    /// according to the run mode in which the motor is operating.
    ///
    /// Setting a power level of zero will brake the motor
    pub fn set_power(&self, power: f64) {
        call_method!(self, self.object, "setPower", "(D)V", [power]);
    }

    /// Sets the power level of the motor, expressed as a fraction of the maximum possible power / speed supported
    /// according to the run mode in which the motor is operating.
    ///
    /// Setting a power level of zero will brake the motor
    #[must_use]
    pub fn get_power(&self) -> f64 {
        call_method!(self, self.object, "getPower", "()D")
            .d()
            .unwrap()
    }

    /// Sets the behavior of the motor when a power level of zero is applied.
    pub fn set_zero_power_behavior(&self, dir: ZeroPowerBehavior) {
        let obj = dir.into_jni_object(&self.env);
        call_method!(
            self,
            self.object,
            "setZeroPowerBehavior",
            format!("(L{};)V", Direction::JNI_CLASS),
            [&obj]
        );
    }

    /// Returns the current behavior of the motor were a power level of zero to be applied.
    pub fn get_zero_power_behavior(&self) -> ZeroPowerBehavior {
        let res = call_method!(
            self,
            self.object,
            "GetZeroPowerBehavior",
            format!("()L{};", Direction::JNI_CLASS)
        )
        .l()
        .unwrap();
        ZeroPowerBehavior::from_jni_object(&self.env, &res)
    }

    /// Sets the desired encoder target position to which the motor should advance or retreat and then actively hold there
    /// at. This behavior is similar to the operation of a servo. The maximum speed at which this advance or retreat occurs
    /// is governed by the power level currently set on the motor. While the motor is advancing or retreating to the desired
    /// target position, `is_busy` will return true.
    /// 
    /// Note that adjustment to a target position is only effective when the motor is in `RunToPosition` `RunMode`. Note
    /// further that, clearly, the motor must be equipped with an encoder in orderfor this mode to function properly.
    pub fn set_target_position(&self, target_pos: i32) {
        call_method!(self, self.object, "setTargetPosition", "(I)V", [target_pos]);
    }

    /// Returns the current target encoder position for this motor.
    #[must_use]
    pub fn get_target_position(&self) -> i32 {
        call_method!(self, self.object, "getTargetPosition", "()I")
            .i()
            .unwrap()
    }

    /// Returns true if the motor is currently advancing or retreating to a target position.
    #[must_use]
    pub fn is_busy(&self) -> bool {
        call_method!(self, self.object, "isBusy", "()Z")
            .z()
            .unwrap()
    }

    /// Returns the current reading of the encoder for this motor. The units for this reading,
    /// that is, the number of ticks per revolution, are specific to the motor/encoder in question,
    /// and thus are not specified here.
    #[must_use]
    pub fn get_current_position(&self) -> i32 {
        call_method!(self, self.object, "getCurrentPosition", "()I")
            .i()
            .unwrap()
    }
}
