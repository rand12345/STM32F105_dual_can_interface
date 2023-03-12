#[derive(Debug)]
pub enum StmError {
    InvalidConfigData,
}
impl core::error::Error for StmError {}
impl core::fmt::Display for StmError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            StmError::InvalidConfigData => write!(f, "Invalid UART config data"),
        }
    }
}
