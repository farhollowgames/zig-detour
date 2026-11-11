//! The integer variant: the navigation mesh and its queries on whole
//! millimetres, with no floating-point step, so every machine gives the
//! same bytes for the same call.
//!
//! The files mirror the float port's, name for name. `doc/integer-design.md`
//! gives the units, the bounds that keep every product inside its type, and
//! how the variant is checked against the float port.

pub const common = @import("integer/common.zig");

pub const Fraction = common.Fraction;

test {
    _ = common;
}
