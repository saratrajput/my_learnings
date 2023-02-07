// use crate::http::Request;
use crate::http::{ParseError, Request, Response, StatusCode};
use std::convert::TryFrom;
use std::convert::TryInto;
use std::io::{Read, Write};
use std::net::TcpListener;

pub trait Handler {
    fn handle_request(&mut self, request: &Request) -> Response;

    fn handle_bad_request(&mut self, e: &ParseError) -> Response {
        println!("Failed to parse a request: {}", e);
        Response::new(StatusCode::BadRequest, None)
    }
}

pub struct Server {
    addr: String,
}

impl Server {
    pub fn new(addr: String) -> Self {
        Self {
            // addr: addr
            addr,
        }
    }

    pub fn run(self, mut handler: impl Handler) {
        println!("Listening on {}", self.addr);

        // Unwrap will return the value if the result is Ok
        // Or terminate the program and log if the result is No
        let listener = TcpListener::bind(&self.addr).unwrap();

        loop {
            match listener.accept() {
                Ok((mut stream, _)) => {
                    let mut buffer = [0; 1024];

                    // In a production server, the following could be a problem if
                    // the incoming data is more than 1024 bytes.
                    match stream.read(&mut buffer) {
                        Ok(_) => {
                            println!("Received a request: {}", String::from_utf8_lossy(&buffer));

                            let response = match Request::try_from(&buffer[..]) {
                                // Ok(request) => {
                                //     dbg!(request);
                                //     // let response = Response::new(StatusCode::NotFound, None);
                                //     // let response = Response::new(
                                //     Response::new(
                                //         StatusCode::Ok,
                                //         Some("<h1> IT WORKS!!!</h1>".to_string()),
                                //         // );
                                //     )
                                //     // write!(stream, "HTTP/1.1 404 Not Found\r\n\r\n");
                                //     // write!(stream, "{}", response);
                                //     // response.send(&mut stream);
                                //     // },
                                // }
                                // Err(e) => {
                                //     println!("Failed to parse a request: {}", e);
                                //     // Response::new(StatusCode::BadRequest, None).send(&mut stream);
                                //     Response::new(StatusCode::BadRequest, None)
                                // }
                                Ok(request) => handler.handle_request(&request),
                                Err(e) => handler.handle_bad_request(&e),
                            };

                            if let Err(e) = response.send(&mut stream) {
                                println!("Failed to send response: {}", e);
                            }
                            // let res: &Result<Request, _> = &buffer[..].try_into();
                        }
                        Err(e) => println!("Failed to read from connection: {}", e),
                    }
                    // let a = 5;
                    // println!("OK");
                }
                Err(e) => println!("Failed to establish a connection: {}", e),
                // _ => println!("Matching all cases.")
            }

            // match is recommended as it can handle many more cases.
            // let res = listener.accept();
            // if res.is_err()
            // {
            //     continue;
            // }
            // let (stream, addr) = res.unwrap();
        }
    }
}
