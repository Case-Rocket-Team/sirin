use crate::{state::State, subsystems::SirinData};

#[derive(Debug, Clone)]
#[non_exhaustive]
pub enum Event {
    DataUpdate(SirinData),
    StateUpdate(State),
}
