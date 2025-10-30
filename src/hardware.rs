//! Hardware-related code.

use std::{rc::Rc, sync::RwLock};

use jni::{objects::JObject, strings::JNIString};

mod devices;
pub use devices::*;

use crate::call_method;

/// A device that can be made from a java object.
pub trait Device<'a, 'local> {
    /// Create a new instance of this type from the java environment and the relevant object.
    fn from_java(env: Rc<RwLock<&'a mut jni::Env<'local>>>, object: JObject<'local>) -> Self;
    /// The Java-formatted class name. Unlike other JNI things, this uses dots.
    const JAVA_CLASS: &'static str;
    /// The JNI-formatted class name. Uses forward slashes instead of dots.
    const JNI_CLASS: &'static str;
}

/// A wrapper for accessing hardware-related methods.
#[derive(Debug)]
pub struct Hardware<'a, 'local> {
    /// The environment.
    pub(crate) env: Rc<RwLock<&'a mut jni::Env<'local>>>,
    /// The actual hardwareMap object. Should be com/qualcomm/robotcore/hardware/HardwareMap.
    pub(crate) hardware_map: JObject<'local>,
}

impl<'a, 'local> Hardware<'a, 'local> {
    /// Get a [`Device`] from the hardware map.
    pub fn get<T: Device<'a, 'local>>(&self, name: impl AsRef<str>) -> T {
        let class = self
            .env
            .write()
            .unwrap()
            .load_class(JNIString::new(T::JAVA_CLASS))
            .unwrap();

        let name = self
            .env
            .write()
            .unwrap()
            .new_string(JNIString::new(name))
            .unwrap();

        let object = self
            .env
            .write()
            .unwrap()
            .call_method(
                &self.hardware_map,
                JNIString::new("get"),
                JNIString::new(format!(
                    "(Ljava/lang/Class;Ljava/lang/String;)L{};",
                    T::JNI_CLASS
                )),
                &[(&class).into(), (&name).into()],
            )
            .unwrap()
            .l()
            .unwrap();

        T::from_java(self.env.clone(), object)
    }
}

/// `DcMotor`s can be configured to internally reverse the values to which, e.g., their motor power is set. This makes
/// it easy to have drive train motors on two sides of a robot: during initialization, one would be set at at forward,
/// the other at reverse, and the difference between the two in that respect could be there after ignored.
///
/// At the start of an `OpMode`, motors are guaranteed to be in the forward direction.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[must_use]
pub enum Direction {
    /// Turn forward. Commonly clockwise.
    #[default]
    Forward,
    /// Turn backward. Commonly counterclockwise.
    Reverse,
}

impl Direction {
    /// The JNI-formatted class name.
    pub const JNI_CLASS: &str = "com/qualcomm/robotcore/hardware/DcMotorSimple$Direction";
    /// The Java-formatted class name.
    pub const JAVA_CLASS: &str = "com.qualcomm.robotcore.hardware.DcMotorSimple.Direction";

    /// Convert this direction into a `JObject`.
    pub fn into_jni_object<'local>(
        self,
        env: &Rc<RwLock<&mut jni::Env<'local>>>,
    ) -> JObject<'local> {
        let class = env
            .write()
            .unwrap()
            .load_class(JNIString::new(
                Self::JNI_CLASS,
            ))
            .unwrap();

        env.write()
            .unwrap()
            .get_static_field(
                class,
                JNIString::new(match self {
                    Self::Forward => "FORWARD",
                    Self::Reverse => "REVERSE",
                }),
                JNIString::new(Self::JNI_CLASS),
            )
            .unwrap()
            .l()
            .unwrap()
    }

    /// Convert this direction into a `JObject`.
    pub fn from_jni_object<'local>(
        env: &Rc<RwLock<&mut jni::Env<'local>>>,
        obj: &JObject<'local>,
    ) -> Self {
        let res = call_method!(env env, obj, "ordinal", "()I").i().unwrap();
        match res {
            0 => Direction::Forward,
            1 => Direction::Reverse,
            _ => unreachable!(),
        }
    }
}

/// `ZeroPowerBehavior` provides an indication as to a motor's behavior when a power level of zero is applied.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[must_use]
pub enum ZeroPowerBehavior {
    /// The behavior of the motor when zero power is applied is not currently known. This value is mostly
    /// useful for your internal state variables. It may not be passed as a parameter to `set_zero_power_behavior`
    /// and will never be returned from `get_zero_power_behavior`.
    Unknown,
    /// The motor stops and then brakes, actively resisting any external force which attempts to turn the motor.
    Brake,
    /// The motor stops and then floats: an external force attempting to turn the motor is not met with active
    /// resistance.
    Float,
}

impl ZeroPowerBehavior {
    /// The JNI-formatted class name.
    pub const JNI_CLASS: &str = "com/qualcomm/robotcore/hardware/DcMotor$Direction";
    /// The Java-formatted class name.
    pub const JAVA_CLASS: &str = "com.qualcomm.robotcore.hardware.DcMotor.Direction";

    /// Convert this direction into a `JObject`.
    pub fn into_jni_object<'local>(
        self,
        env: &Rc<RwLock<&mut jni::Env<'local>>>,
    ) -> JObject<'local> {
        let class = env
            .write()
            .unwrap()
            .load_class(JNIString::new(
                Self::JNI_CLASS,
            ))
            .unwrap();

        env.write()
            .unwrap()
            .get_static_field(
                class,
                JNIString::new(match self {
                    Self::Unknown => "UNKNOWN",
                    Self::Brake => "BRAKE",
                    Self::Float => "FLOAT",
                }),
                JNIString::new(Self::JNI_CLASS),
            )
            .unwrap()
            .l()
            .unwrap()
    }

    /// Convert this direction into a `JObject`.
    pub fn from_jni_object<'local>(
        env: &Rc<RwLock<&mut jni::Env<'local>>>,
        obj: &JObject<'local>,
    ) -> Self {
        let res = call_method!(env env, obj, "ordinal", "()I").i().unwrap();
        match res {
            0 => Self::Unknown,
            1 => Self::Brake,
            2 => Self::Float,
            _ => unreachable!(),
        }
    }
}

/// The run mode of a motor controls how the motor interprets its parameter settings passed through power- and
/// encoder-related methods. Some of these modes internally use `PIDcontrol` to achieve their function, while others
/// do not. Those that do are referred to as "PID modes".
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum RunMode {
    /// The motor is simply to run at whatever velocity is achieved by apply a particular power level to the
    /// motor.
    RunWithoutEncoder,
    /// The motor is to do its best to run at targeted velocity. An encoder must be affixed to the motor in
    /// order to use this mode. This is a PID mode.
    RunUsingEncoder,
    /// The motor is to attempt to rotate in whatever direction is necessary to cause the encoder reading to advance
    /// or retreat from its current setting to the setting which has been provided through the `set_target_position`
    /// method. An encoder must be affixed to this motor in order to use this mode. This is a PID mode.
    RunToPosition,
    /// The motor is to set the current encoder position to zero. In contrast to `RunToPosition`, the motor is not
    /// rotated in order to achieve this; rather, the current rotational position of the motor is simply reinterpreted
    /// as the new zero value. However, as a side effect of placing a motor in this mode, power is removed from the
    /// motor, causing it to stop, though it is unspecified whether the motor enters brake or float mode. Further, it
    /// should be noted that setting a motor to `StopAndResetEncoder` may or may not be a transient state: motors connected
    /// to some motor controllers will remain in this mode until explicitly transitioned to a different one, while motors
    /// connected to other motor controllers will automatically transition to a different mode after the reset of the encoder
    /// is complete.
    StopAndResetEncoder,
}
