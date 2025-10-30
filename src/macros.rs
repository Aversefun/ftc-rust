//! Convinence macros.

/// Call a java method.
#[macro_export]
macro_rules! call_method {
    (env $env:expr, $obj:expr, $name:expr, $sig:expr, $args:expr $(,)?) => {{
        $env
            .write()
            .unwrap()
            .call_method(
                &($obj),
                $crate::jni::strings::JNIString::new($name),
                $crate::jni::strings::JNIString::new($sig),
                &$args.into_iter().map(|v| v.into()).collect::<Vec<$crate::jni::JValue>>(),
            )
            .unwrap()
    }};

    (env $env:expr, $obj:expr, $name:expr, $sig:expr $(,)?) => {{
        $env
            .write()
            .unwrap()
            .call_method(
                &($obj),
                $crate::jni::strings::JNIString::new($name),
                $crate::jni::strings::JNIString::new($sig),
                &[],
            )
            .unwrap()
    }};
    ($self:expr, $obj:expr, $name:expr, $sig:expr, $args:expr $(,)?) => {{
        let this = $self;

        $crate::call_method!(env this.env, $obj, $name, $sig, $args)
    }};
    ($self:expr, $obj:expr, $name:expr, $sig:expr $(,)?) => {{
        let this = $self;

        $crate::call_method!(env this.env, $obj, $name, $sig)
    }};
}