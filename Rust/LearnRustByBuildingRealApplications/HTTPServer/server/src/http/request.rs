use super::method::Method;
pub struct Request
{
    path: String,
    query_string: Option<String>,
    // method: super::method::Method,
    method: Method,
}

impl TryFrom<&[u8]> for Request{
    type Error = String;

    fn try_from(buf: &[u8]) -> Result<Self, Self::Error>{
        unimplemented!()
    }
}
