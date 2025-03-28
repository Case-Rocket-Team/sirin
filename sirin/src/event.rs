use serde::{Deserialize, Serialize};

use crate::measurement::Measurement;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[non_exhaustive]
pub enum Event {
    StateUpdate
}
