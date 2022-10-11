// vec1.rs


fn main()
{
    let mut v = Vec::new();
    v.push(10);
    v.push(20);
    v.push(30);

    let first = v[0]; // will panic if out-of-range
    let maybe_first = v.get(0);

    println!("v is {:?}", v);
    println!("first is {}", first);
    println!("maybe_first is {:?}", maybe_first);
}
// v is [10, 20, 30]
// first is 10
// maybe_first is Some(10)
