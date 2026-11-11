//! The result of a navigation call: one of failure, success or in progress,
//! with detail flags saying why or how completely.

/// A call's outcome, in the bit layout of upstream Detour's `dtStatus`, so
/// `@bitCast` to and from `u32` gives the same numbers.
pub const Status = packed struct(u32) {
    /// The input data is not recognized.
    wrong_magic: bool = false,
    /// The input data is of another version.
    wrong_version: bool = false,
    /// The operation ran out of memory.
    out_of_memory: bool = false,
    /// An input parameter was invalid.
    invalid_param: bool = false,
    /// The result buffer was too small to hold all results.
    buffer_too_small: bool = false,
    /// The query ran out of nodes during its search.
    out_of_nodes: bool = false,
    /// The query did not reach the end location, returning its best guess.
    partial_result: bool = false,
    /// A tile has already been assigned to the given x, y coordinate.
    already_occupied: bool = false,
    reserved: u21 = 0,
    /// The operation is still running.
    in_progress: bool = false,
    succeeded: bool = false,
    failed: bool = false,

    pub const success: Status = .{ .succeeded = true };
    pub const failure: Status = .{ .failed = true };

    /// The detail flags, without the outcome bits.
    pub const detail_mask: u32 = 0x0ffffff;

    /// Both statuses' bits together.
    pub inline fn with(a: Status, b: Status) Status {
        return @bitCast(@as(u32, @bitCast(a)) | @as(u32, @bitCast(b)));
    }

    /// Whether any of `b`'s bits are set in `a`.
    pub inline fn has(a: Status, b: Status) bool {
        return (@as(u32, @bitCast(a)) & @as(u32, @bitCast(b))) != 0;
    }

    /// Only the detail flags of `a`.
    pub inline fn detail(a: Status) Status {
        return @bitCast(@as(u32, @bitCast(a)) & detail_mask);
    }
};
