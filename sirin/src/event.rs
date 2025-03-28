use serde::{Deserialize, Serialize};

use crate::measurement::Measurement;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Event {
    Measurement(Measurement)
}
