use std::collections::HashMap;

// Example query: a=1&b=2&c&d=&e===&d=7&d=abc

#[derive(Debug)]
pub struct QueryString<'buf> {
    data: HashMap<&'buf str, Value<'buf>>,
}

// We need two types: one for simple strings like a=1
// and an array for multiple values with same key like d=[&, 7, abc]
#[derive(Debug)]
pub enum Value<'buf> {
    Single(&'buf str),
    Multiple(Vec<&'buf str>),
}

// Function to read keys from the hash map.
// Returns an Option, if it has the key or None if the key does not exist.
impl<'buf> QueryString<'buf> {
    pub fn get(&self, key: &str) -> Option<&Value> {
        self.data.get(key)
    }
}

// We use the ```From``` trait and not the ```TryFrom``` trait, as this
// conversion cannot fail.
// Example query: a=1&b=2&c&d=&e===&d=7&d=abc
impl<'buf> From<&'buf str> for QueryString<'buf> {
    fn from(s: &'buf str) -> Self {
        let mut data = HashMap::new();

        for sub_str in s.split('&') {
            let mut key = sub_str;
            let mut val = "";
            // Look for the '=' equal sign.
            if let Some(i) = sub_str.find('=') {
                key = &sub_str[..i];  // Everything before the '=' sign.
                val = &sub_str[i+1..]; // Everything after the '=' sign.
            }
            // Case-1: New key, no existing value. : .or_insert()
            // Case-2: Existing key, new value, turn it inot Multiple from Single
            // and append values to the array. : .and_modify()
            // Case-3: Existing key, new value, already Multiple, append values
            // to the array.
            data.entry(key).and_modify(|existing: &mut Value| match existing {
                Value::Single(prev_val) => {
                    // let mut vec = Vec::new();
                    // vec.push(val);
                    // vec.push(prev_val);
                    // let mut vec = vec![prev_val, val];
                    // Follow the pointer and write the new value over whatever
                    // it was pointing to before.
                    *existing = Value::Multiple(vec![prev_val, val]);
                }
                Value::Multiple(vec) => vec.push(val)
            }).or_insert(Value::Single(val));
        }
        QueryString {data}
    }
}
