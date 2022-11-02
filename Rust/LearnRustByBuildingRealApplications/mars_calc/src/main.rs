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

    println!("Enter your weight (kg): ");
    let mut input = String::new();
    // Unwrap: If the result is error -> Terminate
    // If successful -> Yield
    io::stdin().read_line(&mut input).unwrap();

    // Trim input string and convert to float
    let weight: f32 = input.trim().parse().unwrap();
    // println!("{}", weight);

    // Useful for debugging the type of variable
    // dbg!(weight);

    // GDB debugging exercise
    // borrow_string(&input);
    // own_string(input);

    println!("Input: {}", input);

    let mars_weight = calculate_weight_on_mars(weight);
    // mars_weight = mars_weight * 1000.0; // Will throw error if mars_weight is immutable
    println!("Weight on Mars: {} kg", mars_weight);

    // calculate_weight_on_mars(100.0);
}

fn calculate_weight_on_mars(weight: f32) -> f32
{
    (weight / 9.81) * 3.711
}

// GDB Debugging exercise
// fn borrow_string(s: &String)
// {
//     println!("{}", s);
// }

// fn own_string(s: String)
// {
//     println!("{}", s);
// }

// Borrow using &
// fn some_fn(s: &mut String)
// {
//     // s.push_str("a"); //Error: References are immutable
//     // Mutable references are OK
// }
