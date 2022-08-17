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
                Ok((stream, _)) => {
                    stream.read();
                }
                Err(e) => println!("Failed to establish a connection: {}", e),
            }
        }
    }
}
