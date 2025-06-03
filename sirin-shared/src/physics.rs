use uunit::{Hectopascals, Meters, WithUnits};

pub fn approx_pressure_altitude(pressure: Hectopascals<f64>) -> Meters<f64> {
    // Pure version:
    //(44307.7 - 11872.4 * pressure.value.powf(0.190284)).with_units()
    // However we do not have access to powf.

    // From the following Mathematica...
    /*
    
    Needs["FunctionApproximations`"];
    k = 535;
    f = 44307.7 - 11872.4*x^(0.190284);
    g1 = Simplify[
    EconomizedRationalApproximation[f, {x, {250, k}, 3, 2}]]
    g2 = Simplify[
    EconomizedRationalApproximation[f, {x, {k, 800}, 3, 2}]]
    g = Piecewise[{{g1, x < k}, {g2, x >= k}}];
    Plot[{f, g}, {x, 250, 800}, PlotLabel -> "Actual vs. Approx"]
    Plot[{f - g}, {x, 250, 800}, PlotLabel -> "Error"]

    */
    // ... we derive
    // g1 = (1.89892*10^9 + 1.0291*10^7 x - 8915.42 x^2 - 3.06496 x^3)/(76083.8 + 938.747 x + 1. x^2)
    // g2 = (5.15203*10^9 + 1.17824*10^7 x - 14626.1 x^2 - 1.99386 x^3)/(225921. + 1609.98 x + 1. x^2)

    let x = pressure.value;

    let x2 = x * x;
    let x3 = x2 * x;

    // For floats, to retain precision it is best to add from smallest to largest.
    // That being said in this case most of these are on the same order of magnitude.
    if x < 535.0 {
        ((-3.06496 * x3 - 8915.42 * x2 + 1.0291e7 * x + 1.89892e9) / (76083.8 + 938.747 * x + x2)).with_units()
    } else {
        ((- 1.99386 * x3 - 14626.1 * x2 + 1.17824e7 * x + 5.15203e9 ) / (225921. + 1609.98 * x + x2)).with_units()
    }
}