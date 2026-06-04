const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    // The public library module. Dependents import it as `@import("astar")`.
    const astar = b.addModule("astar", .{
        .root_source_file = b.path("src/astar.zig"),
        .target = target,
        .optimize = optimize,
    });

    // `zig build test` runs the unit tests embedded in src/astar.zig.
    const tests = b.addTest(.{ .root_module = astar });
    const run_tests = b.addRunArtifact(tests);
    const test_step = b.step("test", "Run the unit tests");
    test_step.dependOn(&run_tests.step);

    // `zig build example` builds and runs the grid pathfinding demo.
    const example_mod = b.createModule(.{
        .root_source_file = b.path("examples/grid.zig"),
        .target = target,
        .optimize = optimize,
    });
    example_mod.addImport("astar", astar);
    const example = b.addExecutable(.{
        .name = "grid-example",
        .root_module = example_mod,
    });
    b.installArtifact(example);
    const run_example = b.addRunArtifact(example);
    const example_step = b.step("example", "Build and run the grid pathfinding example");
    example_step.dependOn(&run_example.step);
}
