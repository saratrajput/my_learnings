#![allow(dead_code)]
// Import modules
mod http;
mod server;
mod website_handler;

// Define namespaces
use http::Method;
use http::Request;
use server::Server;
use website_handler::WebsiteHandler;

fn main() {
    // let server = server::Server::new("127.0.0.1:8080".to_string());
    let server = Server::new("127.0.0.1:8080".to_string());
    // server.run();
    server.run(WebsiteHandler);
}

/*
GET /user?id=10 HTTP/1.1\r\n
HEADERS \r\n
BODY
*/
