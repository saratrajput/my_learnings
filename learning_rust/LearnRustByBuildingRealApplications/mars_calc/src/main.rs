use std::io;


fn main() 
{
    // println!("Hello, world!");
    // println!("Number: {}", 100);
    // println!("Number: {}, String: {}", 100, "abcd");

    // All variables in Rust are immutable by default
    // Make it mutable by appending mut to variable name
    // let mut mars_weight = calculate_weight_on_mars(100.0);

    
    // Error: We can't have both mutable and immutable borrows
    // let mut s1 = &mut input;
    // let s2 = &input;
    // println!("{} {}", s1, s2);
    
    // some_fn(&mut input); // This is not valid since once the function ends the value is destroyed
    // let mut s = input; // The variable input is no longer available after this
    // This is valid since the value is being copied
    // let a = 5;
    // let b = a;

    let mut input = String::new();
    io::stdin().read_line(&mut input);

    borrow_string(&input);
    own_string(input);
    // println!("Input: {}", input);

    // let mars_weight = calculate_weight_on_mars(100.0);
    // mars_weight = mars_weight * 1000.0; // Will throw error if mars_weight is immutable
    // println!("Weight on Mars: {} kg", mars_weight);

    // calculate_weight_on_mars(100.0);
}

fn calculate_weight_on_mars(weight: f32) -> f32
{
    (weight / 9.81) * 3.711
}

fn borrow_string(s: &String)
{
    println!("{}", s);
}

fn own_string(s: String)
{
    println!("{}", s);
}

// Borrow using &
// fn some_fn(s: &mut String)
// {
//     // s.push_str("a"); //Error: References are immutable
//     // Mutable references are OK
// }