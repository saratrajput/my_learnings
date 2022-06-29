fn main()
{
    let server = Server::new("127.0.0.1:8080".to_string());
    server.run();
}

struct Server
{
    addr: String,
}

impl Server
{
    fn new(addr: String) -> Self
    {
        Self
        {
            // addr: addr
            addr
        }
    }

    fn run(self)
    {
        println!("Listening on {}", self.addr);
    }
}


struct Request
{
    path: String,
    query_string: Option<String>,
    method: Method,
}


enum Method
{
    GET,
    DELETE,
    POST,
    PUT,
    HEAD,
    CONNECT,
    OPTIONS,
    TRACE,
    PATCH,
}

/*
GET /user?id=10 HTTP/1.1\r\n
HEADERS \r\n
BODY
*/
