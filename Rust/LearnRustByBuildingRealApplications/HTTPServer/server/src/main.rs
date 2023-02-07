#![allow(dead_code)]
// Import modules
mod http;
mod server;
mod website_handler;

// Define namespaces
use http::Method;
use http::Request;
use server::Server;
use std::env;
use website_handler::WebsiteHandler;

fn main() {
    let default_path = format!("{}/public", env!("CARGO_MANIFEST_DIR"));
    // let public_path = env::var("PUBLIC_PATH").unwrap();
    let public_path = env::var("PUBLIC_PATH").unwrap_or(default_path);
    println!("public path: {}", public_path);
    // let server = server::Server::new("127.0.0.1:8080".to_string());
    let server = Server::new("127.0.0.1:8080".to_string());
    // server.run();
    // server.run(WebsiteHandler);
    server.run(WebsiteHandler::new(public_path));
}

/*
GET /user?id=10 HTTP/1.1\r\n
HEADERS \r\n
BODY
*/
