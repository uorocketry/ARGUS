#![no_std]
//!
//! This crate contains code to convert type k thermocouple voltages to temperatures.
//!

///Ranges where polynomial coefficients are for
pub const ENERGY_RANGES: [[f64; 2]; 3] = [[-5.891, 0.0], [0.0, 20.644], [20.644, 54.886]];

///Type K thermocouple coefficients for polynomial calculation
/// See https://www.eevblog.com/forum/metrology/a-dive-into-k-type-thermocouple-maths/
pub const TYPE_K_COEF: [[f64; 10]; 3] = [
    [
        0.0,
        25.173462,
        -1.1662878,
        -1.0833638,
        -0.89773540,
        -0.37342377,
        -0.086632643,
        -0.010450598,
        -0.00051920577,
        0.0,
    ],
    [
        0.0,
        25.08355,
        0.07860106,
        -0.2503131,
        0.08315270,
        -0.01228034,
        0.0009804036,
        -0.00004413030,
        0.000001057734,
        -0.00000001052755,
    ],
    [
        -131.8058,
        48.30222,
        -1.646031,
        0.05464731,
        -0.0009650715,
        0.000008802193,
        -0.00000003110810,
        0.0,
        0.0,
        0.0,
    ],
];

///function for power
/// in: base: f64, exp: i32
/// out: result: f64
fn pow(base: f64, exp: i32) -> f64 {
    let mut result = 1.0;
    if exp < 0 {
        result = 1.0 / pow(base, -exp);
    } else {
        for _ in 0..exp {
            result *= base;
        }
    }
    return result;
}

///function to calculate the conversion between voltage to celsius from the thermocoupler.
///in: voltage: f64
///out: celsius: f64
pub fn voltage_to_celsius(voltage: f64) -> f64 {
    //define variables
    let mut result = 0.0;
    let mut i = 0;

    //goes through the different ranges
    while i < ENERGY_RANGES.len() {
        if voltage >= ENERGY_RANGES[i][0] && voltage <= ENERGY_RANGES[i][1] {
            //calculates the result
            for k in 0..TYPE_K_COEF[i].len() {
                result += TYPE_K_COEF[i][k] * pow(voltage, k as i32);
            }
            return result;
        } else {
            //if the voltage is not in the range, it goes to the next range
            i += 1;
        }
    }

    return -1.0;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::voltage_to_celsius;

    #[test]
    fn voltage_to_celsius_test1() {
        //println!("Test 1: {}", voltage_to_celsius(20.644));
        let result: f64 = voltage_to_celsius(20.644);
        assert!(499.97 <= result && 500.0 >= result);

        // println!("Test 2: {}", voltage_to_celsius(6.138));
        let result: f64 = voltage_to_celsius(6.138);
        assert!(150.01 <= result && 150.03 >= result);

        // println!("Test 3: {}", voltage_to_celsius(0.039));
        let result: f64 = voltage_to_celsius(0.039);
        assert!(0.97 <= result && 0.98 >= result);

        // println!("Test 4: {}", voltage_to_celsius(-0.778));
        let result: f64 = voltage_to_celsius(-0.778);
        assert!(-20.03 <= result && -20.01 >= result);

        // println!("Test 5: {}", voltage_to_celsius(10.0));
        let result: f64 = voltage_to_celsius(10.0);
        assert!(246.1 <= result && 246.3 >= result);
    }
}
