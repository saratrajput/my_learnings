// fun2.rs
fn abs(x: f64) -> f64
{
    if x > 0.0
    {
        x
    }
    else
    {
        -x
    }
}

// ensure the number always falls in the given range
fn clamp(x: f64, x1: f64, x2: f64) -> f64
{
    if x < x1
    {
        x1
    }
    else if x > x2
    {
        x2
    }
    else
    {
        x
    }
}


fn factorial(n: u64) -> u64
{
    if n == 0
    {
        1
    }
    else
    {
        n * factorial(n-1)
    }
}

fn main()
{
    let res = abs(-2.0);
    println!("abs is {}", res);
}
