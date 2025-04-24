use crate::song::ToSongError;
use embassy_sync::pubsub::Error as PubSubError;
use rfm9::Rfm9Error;

macro_rules! join_error {
    (enum $err:ident ($($specialized_err:ident),*)) => {
        // Every possible Sirin error in one Big Beautiful Enum
        #[derive(Debug, Clone)]
        pub enum $err {
            $(
                $specialized_err($specialized_err)
            ),*
        }

        $(
            impl From<$specialized_err> for $err {
                fn from(value: $specialized_err) -> Self {
                    Self::$specialized_err(value)
                }
            }
        )*
    };
}

join_error!(enum SirinError (
    PubSubError,
    ToSongError,
    Rfm9Error
));