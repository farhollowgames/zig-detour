const std = @import("std");

/// The build switches of the original the port keeps.
const Configuration = struct {
    /// Ends the names of the artifacts built for it, so two configurations
    /// build side by side.
    suffix: []const u8 = "",
    /// Polygon and tile references of 64 bits (DT_POLYREF64).
    polyref64: bool = false,
    /// Query filters that replace their two functions (DT_VIRTUAL_QUERYFILTER).
    virtual_query_filter: bool = false,
    /// No checks for infinite and NaN inputs (RC_FAST_MATH).
    fast_math: bool = false,
};

/// The tests run with every switch off, then with every switch on.
const test_configurations = [_]Configuration{
    .{},
    .{ .suffix = "-switches", .polyref64 = true, .virtual_query_filter = true, .fast_math = true },
};

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const chosen: Configuration = .{
        .polyref64 = b.option(
            bool,
            "polyref64",
            "Polygon and tile references of 64 bits instead of 32 (default false)",
        ) orelse false,
        .virtual_query_filter = b.option(
            bool,
            "virtual-query-filter",
            "Let a query filter replace its pass and cost functions (default false)",
        ) orelse false,
        .fast_math = b.option(
            bool,
            "fast-math",
            "Skip the checks for infinite and NaN inputs (default false)",
        ) orelse false,
    };

    const detour = b.addModule("detour", .{
        .root_source_file = b.path("src/detour.zig"),
        .target = target,
        .optimize = optimize,
    });
    detour.addImport("build_options", optionsModule(b, chosen));

    // Tests default to ReleaseSafe: the whole-mesh searches take long in
    // Debug, and ReleaseSafe keeps every assert and overflow check on.
    const test_optimize = b.option(
        std.lang.Optimize,
        "test-optimize",
        "Optimize mode for the tests (default ReleaseSafe)",
    ) orelse .safe;
    const test_filters = b.option(
        []const []const u8,
        "test-filter",
        "Skip tests that do not match any filter",
    ) orelse &.{};

    const fuzz_seed = b.option(
        u64,
        "fuzz-seed",
        "Move every seeded differential test to other inputs (default 0: the tests' own seeds)",
    ) orelse 0;

    const test_step = b.step("test", "Run unit and differential tests");
    const bench_step = b.step("bench", "Time the port against the original (ReleaseFast)");

    for (test_configurations) |configuration| {
        const unit_tests = b.addTest(.{
            .name = b.fmt("unit{s}", .{configuration.suffix}),
            .root_module = detourModule(b, target, test_optimize, optionsModule(b, configuration)),
            .filters = test_filters,
        });
        test_step.dependOn(&b.addRunArtifact(unit_tests).step);
    }

    // The original is fetched only to test and time this package itself,
    // never for a package that depends on it.
    if (!b.isRoot()) return;
    const upstream = b.lazyDependency("recastnavigation", .{}) orelse return;
    const recast = recastLibrary(b, upstream, target);

    for (test_configurations) |configuration| {
        const options = optionsModule(b, configuration);
        const detour_for_tests = detourModule(b, target, test_optimize, options);
        const differential = b.addTest(.{
            .name = b.fmt("differential{s}", .{configuration.suffix}),
            .root_module = b.createModule(.{
                .root_source_file = b.path("test/differential.zig"),
                .target = target,
                .optimize = test_optimize,
                .link_libc = true,
                .imports = &.{
                    .{ .name = "detour", .module = detour_for_tests },
                    .{ .name = "build_options", .module = options },
                    .{ .name = "fuzz_options", .module = fuzzOptionsModule(b, fuzz_seed) },
                },
            }),
            .filters = test_filters,
        });
        differential.root_module.linkLibrary(
            upstreamLibrary(b, upstream, target, configuration, true),
        );
        differential.root_module.linkLibrary(recast);
        addMeshes(b, upstream, differential.root_module);
        addIntegerSources(b, differential.root_module);
        test_step.dependOn(&b.addRunArtifact(differential).step);
    }

    const bench_options = optionsModule(b, .{});
    const bench = b.addExecutable(.{
        .name = "bench",
        .root_module = b.createModule(.{
            .root_source_file = b.path("tools/bench.zig"),
            .target = target,
            .optimize = .fast,
            .link_libc = true,
        }),
    });
    // The tests' meshes and their bindings to the original, as one module.
    const support = b.createModule(.{
        .root_source_file = b.path("test/support.zig"),
        .target = target,
        .optimize = .fast,
        .imports = &.{
            .{ .name = "detour", .module = detourModule(b, target, .fast, bench_options) },
            .{ .name = "build_options", .module = bench_options },
            // The benchmark's inputs stay those of the tests' own seeds.
            .{ .name = "fuzz_options", .module = fuzzOptionsModule(b, 0) },
        },
    });
    addMeshes(b, upstream, support);
    bench.root_module.addImport("support", support);
    bench.root_module.linkLibrary(upstreamLibrary(b, upstream, target, .{}, false));
    bench.root_module.linkLibrary(recast);
    const run_bench = b.addRunArtifact(bench);
    run_bench.addPassthruArgs();
    bench_step.dependOn(&run_bench.step);
}

fn optionsModule(b: *std.Build, configuration: Configuration) *std.Build.Module {
    const options = b.addOptions();
    options.addOption(bool, "polyref64", configuration.polyref64);
    options.addOption(bool, "virtual_query_filter", configuration.virtual_query_filter);
    options.addOption(bool, "fast_math", configuration.fast_math);
    return options.createModule();
}

/// The seed offset of the differential tests' random inputs.
fn fuzzOptionsModule(b: *std.Build, fuzz_seed: u64) *std.Build.Module {
    const options = b.addOptions();
    options.addOption(u64, "fuzz_seed", fuzz_seed);
    return options.createModule();
}

fn detourModule(
    b: *std.Build,
    target: std.Build.ResolvedTarget,
    optimize: std.lang.Optimize,
    options: *std.Build.Module,
) *std.Build.Module {
    return b.createModule(.{
        .root_source_file = b.path("src/detour.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{.{ .name = "build_options", .module = options }},
    });
}

const UpstreamLibrary = struct {
    directory: []const u8,
    sources: []const []const u8,
};

const upstream_libraries = [_]UpstreamLibrary{
    .{ .directory = "Detour", .sources = &.{
        "DetourAlloc.cpp",
        "DetourAssert.cpp",
        "DetourCommon.cpp",
        "DetourNavMesh.cpp",
        "DetourNavMeshQuery.cpp",
        "DetourNode.cpp",
    } },
    .{ .directory = "DetourCrowd", .sources = &.{
        "DetourCrowd.cpp",
        "DetourLocalBoundary.cpp",
        "DetourObstacleAvoidance.cpp",
        "DetourPathCorridor.cpp",
        "DetourPathQueue.cpp",
        "DetourProximityGrid.cpp",
    } },
    .{ .directory = "DetourTileCache", .sources = &.{
        "DetourTileCache.cpp",
        "DetourTileCacheBuilder.cpp",
    } },
};

const recast_sources = [_][]const u8{
    "Recast.cpp",
    "RecastAlloc.cpp",
    "RecastArea.cpp",
    "RecastAssert.cpp",
    "RecastContour.cpp",
    "RecastFilter.cpp",
    "RecastLayers.cpp",
    "RecastMesh.cpp",
    "RecastMeshDetail.cpp",
    "RecastRasterization.cpp",
    "RecastRegion.cpp",
};

/// The C interfaces the tests call the original through.
const shim_sources = [_][]const u8{
    "detour_shim.cpp",
    "crowd_shim.cpp",
    "tile_cache_shim.cpp",
    "recast_shim.cpp",
};

/// Fused multiply-adds are off so float steps round as written, as Zig's
/// do. The original's asserts are off: the port's own asserts, on in the
/// tests, stand at the same places.
const cpp_flags = [_][]const u8{
    "-std=c++17",
    "-ffp-contract=off",
    "-fno-fast-math",
    "-fno-exceptions",
    "-fno-rtti",
    "-fno-threadsafe-statics",
    "-fno-sanitize=undefined",
    "-DNDEBUG",
    "-DRC_DISABLE_ASSERTS",
};

/// The original Detour, DetourCrowd and DetourTileCache with the tests' C
/// interface, built with `configuration`'s switches.
///
/// The navigation mesh builder sorts with the C library's `qsort`, which
/// may order equal items any way. `oracle` gives it a stable sort instead,
/// the order the port's sort gives, so the tests can compare bytes. Without
/// `oracle` it is built as it ships, at `-O3`, for the benchmark.
fn upstreamLibrary(
    b: *std.Build,
    upstream: *std.Build.Dependency,
    target: std.Build.ResolvedTarget,
    configuration: Configuration,
    oracle: bool,
) *std.Build.Step.Compile {
    const lib = b.addLibrary(.{
        .name = b.fmt("detour_{s}{s}", .{
            if (oracle) "oracle" else "upstream",
            configuration.suffix,
        }),
        .linkage = .static,
        .root_module = b.createModule(.{
            .target = target,
            .optimize = .fast,
            .link_libc = true,
            // For the C++ standard headers the original includes.
            .link_libcpp = true,
        }),
    });
    const module = lib.root_module;

    var flags: std.ArrayList([]const u8) = .empty;
    flags.appendSlice(b.allocator, &cpp_flags) catch @panic("OOM");
    if (configuration.polyref64) flags.append(b.allocator, "-DDT_POLYREF64") catch @panic("OOM");
    if (configuration.virtual_query_filter) {
        flags.append(b.allocator, "-DDT_VIRTUAL_QUERYFILTER") catch @panic("OOM");
    }
    if (configuration.fast_math) flags.append(b.allocator, "-DRC_FAST_MATH") catch @panic("OOM");
    if (!oracle) flags.append(b.allocator, "-O3") catch @panic("OOM");

    for (upstream_libraries) |library| {
        module.addIncludePath(upstream.path(b.fmt("{s}/Include", .{library.directory})));
        module.addCSourceFiles(.{
            .root = upstream.path(b.fmt("{s}/Source", .{library.directory})),
            .files = library.sources,
            .flags = flags.items,
        });
    }
    module.addIncludePath(upstream.path("Recast/Include"));
    module.addCSourceFiles(.{
        .root = b.path("test/shim"),
        .files = &shim_sources,
        .flags = flags.items,
    });

    var builder_flags: std.ArrayList([]const u8) = .empty;
    builder_flags.appendSlice(b.allocator, flags.items) catch @panic("OOM");
    // `ref_stable_qsort` is one of the shim's functions.
    if (oracle) builder_flags.append(b.allocator, "-Dqsort=ref_stable_qsort") catch @panic("OOM");
    module.addCSourceFiles(.{
        .root = upstream.path("Detour/Source"),
        .files = &.{"DetourNavMeshBuilder.cpp"},
        .flags = builder_flags.items,
    });

    return lib;
}

/// The original Recast, which builds the meshes the tests navigate.
fn recastLibrary(
    b: *std.Build,
    upstream: *std.Build.Dependency,
    target: std.Build.ResolvedTarget,
) *std.Build.Step.Compile {
    const lib = b.addLibrary(.{
        .name = "recast_upstream",
        .linkage = .static,
        .root_module = b.createModule(.{
            .target = target,
            .optimize = .fast,
            .link_libc = true,
            .link_libcpp = true,
        }),
    });
    lib.root_module.addIncludePath(upstream.path("Recast/Include"));
    lib.root_module.addCSourceFiles(.{
        .root = upstream.path("Recast/Source"),
        .files = &recast_sources,
        .flags = &cpp_flags,
    });
    return lib;
}

/// The integer variant's sources, which a test reads to find no float in
/// them, by the names it embeds them under.
const integer_sources = [_][2][]const u8{
    .{ "integer_source_root", "src/integer.zig" },
    .{ "integer_source_common", "src/integer/common.zig" },
    .{ "integer_source_nav_mesh", "src/integer/nav_mesh.zig" },
    .{ "integer_source_node", "src/integer/node.zig" },
    .{ "integer_source_nav_mesh_query", "src/integer/nav_mesh_query.zig" },
    .{ "integer_source_path_corridor", "src/integer/path_corridor.zig" },
};

fn addIntegerSources(b: *std.Build, module: *std.Build.Module) void {
    for (integer_sources) |source| {
        module.addAnonymousImport(source[0], .{ .root_source_file = b.path(source[1]) });
    }
}

const mesh_names = [_][]const u8{ "dungeon", "nav_test", "undulating" };

/// RecastDemo's level meshes, embedded by name (`@embedFile("dungeon")`).
fn addMeshes(b: *std.Build, upstream: *std.Build.Dependency, module: *std.Build.Module) void {
    for (mesh_names) |name| {
        module.addAnonymousImport(name, .{
            .root_source_file = upstream.path(b.fmt("RecastDemo/Bin/Meshes/{s}.obj", .{name})),
        });
    }
}
