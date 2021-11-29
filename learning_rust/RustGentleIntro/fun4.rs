// fun4.rs

fn modifies(x: &mut f64)
{
    *x = 2.0;
}

fn main()
{
    let mut res = 0.0;
    modifies(&mut res);
    println!("res is {}", res);
}
