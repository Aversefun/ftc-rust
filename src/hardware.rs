//! Hardware-related code.

use jni::{Env, JavaVM, objects::{JClass, JObject}, refs::Global, strings::JNIString};

mod devices;
pub use devices::*;

use crate::{call_method, new_global, new_string};

/// A device that can be made from a java object.
pub trait Device {
    /// Create a new instance of this type from the java environment and the relevant object.
    fn from_java(vm: JavaVM, object: Global<JObject<'static>>) -> Self;
    /// The Java-formatted class name. Unlike other JNI things, this uses dots.
    const JAVA_CLASS: &'static str;
    /// The JNI-formatted class name. Uses forward slashes instead of dots.
    const JNI_CLASS: &'static str;
}

/// A wrapper for accessing hardware-related methods.
#[derive(Debug)]
#[doc(alias = "HardwareMap")]
pub struct Hardware {
    /// The environment.
    pub(crate) vm: JavaVM,
    /// The actual hardwareMap object. Should be com/qualcomm/robotcore/hardware/HardwareMap.
    pub(crate) hardware_map: Global<JObject<'static>>,
}

impl Hardware {
    /// Get a [`Device`] from the hardware map.
    pub fn get<T: Device>(&self, name: impl AsRef<str>) -> T {
        let object = self
            .vm
            .attach_current_thread(|env| {
                let class = env.load_class(JNIString::new(T::JAVA_CLASS)).unwrap();
                let name = new_string!(env env, name).unwrap();

                new_global!(env, env.call_method(
                    &self.hardware_map,
                    JNIString::new("get"),
                    JNIString::new(format!(
                        "(Ljava/lang/Class;Ljava/lang/String;)L{};",
                        T::JNI_CLASS
                    )),
                    &[(&class).into(), (&name).into()],
                )
                .unwrap()
                .l().unwrap())
            })
            .unwrap();

        T::from_java(self.vm.clone(), object)
    }
}

/// Get a `JClass` of the provided type.
fn get_class<'local, T: IntoJniObject>(env: &mut Env<'local>) -> JClass<'local> {
    env
        .load_class(JNIString::new(
            T::JNI_CLASS,
        ))
        .unwrap()
}

/// Generate an implementation of `IntoJniObject` for an enum.
macro_rules! enum_variant_into {
    {
        $ty:ty,
        $jni_class:literal,
        $java_class:literal,
        $($variant:ident),*
        $(,)?
    } => {
        impl IntoJniObject for $ty {
            const JNI_CLASS: &'static str = $jni_class;
            const JAVA_CLASS: &'static str = $java_class;
            fn into_jni_object<'local>(self, env: &mut Env<'local>) -> JObject<'local> {
                let class = get_class::<Self>(env);
                env
                    .get_static_field(
                        class,
                        JNIString::new(match self {
                            $(Self:: $variant => stringify!($variant).to_uppercase()),*
                        }),
                        JNIString::new(Self::JNI_CLASS),
                    )
                    .unwrap()
                    .l()
                    .unwrap()
            }

            fn from_jni_object(
                vm: JavaVM,
                obj: &JObject,
            ) -> Self {
                let res = vm.attach_current_thread(|env| call_method!(env env, obj, "ordinal", "()I", []).unwrap().i()).unwrap();
                let mut items = vec![$(Self:: $variant),*];
                let full_len = items.len();
                match res {
                    $(x if x == {items.pop(); (full_len - items.len()) as i32} => Self:: $variant),*,
                    _ => unreachable!()
                }
            }
        }
    };
}

/// Convert this type into a JNI object.
pub trait IntoJniObject {
    /// The JNI-formatted class name.
    const JNI_CLASS: &'static str;
    /// The Java-formatted class name.
    const JAVA_CLASS: &'static str;

    /// Convert this type into a `JObject`.
    fn into_jni_object<'local>(self, env: &mut Env<'local>) -> JObject<'local>;
    /// Convert a `JObject` into this type.
    fn from_jni_object(vm: JavaVM, obj: &JObject) -> Self;
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

enum_variant_into! {
    Direction,
    "com/qualcomm/robotcore/hardware/DcMotorSimple$Direction",
    "com.qualcomm.robotcore.hardware.DcMotorSimple.Direction",
    Forward,
    Reverse,
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

enum_variant_into! {
    ZeroPowerBehavior,
    "com/qualcomm/robotcore/hardware/DcMotor$ZeroPowerBehavior",
    "com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior",
    Unknown,
    Brake,
    Float,
}

/// The run mode of a motor controls how the motor interprets its parameter settings passed through power- and
/// encoder-related methods. Some of these modes internally use `PIDcontrol` to achieve their function, while others
/// do not. Those that do are referred to as "PID modes".
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[must_use]
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

enum_variant_into! {
    RunMode,
    "com/qualcomm/robotcore/hardware/DcMotor$RunMode",
    "com.qualcomm.robotcore.hardware.DcMotor.RunMode",
    RunWithoutEncoder,
    RunUsingEncoder,
    RunToPosition,
    StopAndResetEncoder,
}

