const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    // Dependencies (without target/optimize - they handle that internally)
    const zglfw = b.dependency("zglfw", .{});
    const zgui = b.dependency("zgui", .{
        .backend = .glfw_opengl3,
    });
    const zopengl = b.dependency("zopengl", .{});
    const zalgebra = b.dependency("zalgebra", .{});
    const chad = b.dependency("chad", .{
        .target = target,
        .optimize = optimize,
    });

    // Get chad module (exposes geometry, octree, objloader)
    const chad_mod = chad.module("chad");

    // Debug visualizer module
    const debug_visualizer_mod = b.addModule("debug_visualizer", .{
        .root_source_file = b.path("debug_visualizer.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "geometry", .module = chad_mod },
            .{ .name = "zopengl", .module = zopengl.module("root") },
            .{ .name = "zalgebra", .module = zalgebra.module("zalgebra") },
        },
    });

    // Octree visualizer executable
    const octree_visualizer_exe = b.addExecutable(.{
        .name = "octree-visualizer",
        .root_module = b.createModule(.{
            .root_source_file = b.path("octree_visualizer.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "debug_visualizer", .module = debug_visualizer_mod },
                .{ .name = "geometry", .module = chad_mod },
                .{ .name = "zglfw", .module = zglfw.module("root") },
                .{ .name = "zgui", .module = zgui.module("root") },
                .{ .name = "zopengl", .module = zopengl.module("root") },
                .{ .name = "zalgebra", .module = zalgebra.module("zalgebra") },
            },
        }),
    });

    // Link libraries
    octree_visualizer_exe.linkLibrary(zglfw.artifact("glfw"));
    octree_visualizer_exe.linkLibrary(zgui.artifact("imgui"));

    // Link OpenGL framework on macOS
    octree_visualizer_exe.linkFramework("OpenGL");

    b.installArtifact(octree_visualizer_exe);

    // Run step
    const run_step = b.step("run", "Run the octree visualizer");
    const run_cmd = b.addRunArtifact(octree_visualizer_exe);
    run_step.dependOn(&run_cmd.step);
    run_cmd.step.dependOn(b.getInstallStep());

    if (b.args) |args| {
        run_cmd.addArgs(args);
    }
}
