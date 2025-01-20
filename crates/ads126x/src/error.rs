use core::fmt::Debug;

#[derive(Debug)]
pub enum ADS126xError {
    IO,
    InvalidInputData,
}
