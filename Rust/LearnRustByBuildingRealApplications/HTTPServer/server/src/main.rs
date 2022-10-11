// Import modules
mod server;
mod http;

// Define namespaces
use server::Server;
use http::Request;
use http::Method;


fn main()
{
    // let server = server::Server::new("127.0.0.1:8080".to_string());
    let server = Server::new("127.0.0.1:8080".to_string());
    server.run();
}

/*
GET /user?id=10 HTTP/1.1\r\n
HEADERS \r\n
BODY
*/
