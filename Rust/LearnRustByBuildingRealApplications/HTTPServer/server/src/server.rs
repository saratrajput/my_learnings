use crate::http::Request;
use std::convert::TryFrom;
use std::convert::TryInto;
use std::io::Read;
use std::net::TcpListener;

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

    pub fn run(self) {
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
                    match stream.read(&mut buffer)
                    {
                        Ok(_) => {
                            println!("Received a request: {}", String::from_utf8_lossy(&buffer));

                            match Request::try_from(&buffer[..])
                            {
                                Ok(request) => {
                                    dbg!(request);
                                },
                                Err(e) => println!("Failed to parse a request: {}", e),
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
