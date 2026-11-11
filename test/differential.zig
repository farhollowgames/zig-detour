//! The differential tests: each file compares one area of the port with the
//! original, compiled from source.

const std = @import("std");
const detour = @import("detour");
const support = @import("support.zig");

comptime {
    _ = support;
    _ = @import("common_test.zig");
    _ = @import("layout_test.zig");
    _ = @import("nav_mesh_test.zig");
    _ = @import("nav_mesh_builder_test.zig");
    _ = @import("determinism_test.zig");
    _ = @import("tile_cache_test.zig");
    _ = @import("proximity_grid_test.zig");
    _ = @import("obstacle_avoidance_test.zig");
    _ = @import("query_point_test.zig");
    _ = @import("node_test.zig");
    _ = @import("query_filter_test.zig");
    _ = @import("query_area_test.zig");
    _ = @import("query_path_test.zig");
    _ = @import("local_boundary_test.zig");
    _ = @import("path_corridor_test.zig");
    _ = @import("tile_cache_builder_test.zig");
    _ = @import("crowd_test.zig");
}

/// Takes the address of every public function under `T`, so the compiler
/// analyses the body of code no test reaches yet.
fn referenceAll(comptime T: type, comptime depth: u32) void {
    inline for (comptime std.meta.declarations(T)) |name| {
        switch (@typeInfo(@TypeOf(@field(T, name)))) {
            .@"fn" => |function| if (!function.is_generic) {
                const pointer: *const anyopaque = @ptrCast(&@field(T, name));
                std.mem.doNotOptimizeAway(pointer);
            },
            .type => if (depth > 0) switch (@typeInfo(@field(T, name))) {
                .@"struct", .@"enum", .@"union", .@"opaque" => {
                    referenceAll(@field(T, name), depth - 1);
                },
                else => {},
            },
            else => {},
        }
    }
}

test "every public declaration compiles" {
    referenceAll(detour, 3);
}

test "a tiled world builds on both sides" {
    var world: support.World = undefined;
    try world.init(std.testing.allocator, support.nav_test_obj, support.tiled_config, .{});
    defer world.deinit();
    try std.testing.expect(world.level.grid.width > 1);
}
