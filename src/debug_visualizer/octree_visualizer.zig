const std = @import("std");
const zglfw = @import("zglfw");
const zopengl = @import("zopengl");
const za = @import("zalgebra");

const chad = @import("geometry");
const geometry = chad.geometry;
const Mesh = geometry.Mesh;
const Model = geometry.Model;
const Aabb = geometry.Aabb;
const Vec3 = geometry.Vec3;

const Octree = chad.octree.Octree;

const debug_viz = @import("debug_visualizer");
const DebugVisualizer = debug_viz.DebugVisualizer;

const ZaVec3 = za.Vec3;
const ZaMat4 = za.Mat4;

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    // Initialize GLFW
    zglfw.init() catch {
        std.log.err("Failed to initialize GLFW", .{});
        return error.GLFWInitFailed;
    };
    defer zglfw.terminate();

    // GL 3.3 + GLSL 330
    zglfw.windowHint(.context_version_major, 3);
    zglfw.windowHint(.context_version_minor, 3);
    zglfw.windowHint(.opengl_profile, .opengl_core_profile);
    zglfw.windowHint(.opengl_forward_compat, true);

    // Create window
    const window = zglfw.Window.create(1280, 720, "Octree Visualizer", null) catch {
        std.log.err("Failed to create window", .{});
        return error.WindowCreationFailed;
    };
    defer window.destroy();

    zglfw.makeContextCurrent(window);
    zglfw.swapInterval(1);

    // Load OpenGL
    zopengl.loadCoreProfile(zglfw.getProcAddress, 3, 3) catch {
        std.log.err("Failed to load OpenGL", .{});
        return error.OpenGLLoadFailed;
    };

    // Load bunny model
    std.log.info("Loading bunny model...", .{});
    var model = try loadBunnyModel(allocator);
    defer model.deinit();
    std.log.info("Loaded model with {d} vertices, {d} triangles", .{
        model.totalVertexCount(),
        model.totalTriangleCount(),
    });

    // Compute bounding box of the model
    const model_bounds = computeModelBounds(&model);
    std.log.info("Model bounds: min=({d:.2}, {d:.2}, {d:.2}), max=({d:.2}, {d:.2}, {d:.2})", .{
        model_bounds.bmin.toArray()[0],
        model_bounds.bmin.toArray()[1],
        model_bounds.bmin.toArray()[2],
        model_bounds.bmax.toArray()[0],
        model_bounds.bmax.toArray()[1],
        model_bounds.bmax.toArray()[2],
    });

    // Build octree
    std.log.info("Building octree...", .{});
    var octree = Octree.init(allocator, model_bounds, .{
        .max_depth = 5,
        .max_primitives_per_node = 8,
    });
    defer octree.deinit();

    // Insert all triangles into the octree
    var prim_id: u32 = 0;
    for (model.meshes.items) |*mesh| {
        var i: usize = 0;
        while (i < mesh.indices.len) : (i += 3) {
            const v0 = mesh.vertices[mesh.indices[i]];
            const v1 = mesh.vertices[mesh.indices[i + 1]];
            const v2 = mesh.vertices[mesh.indices[i + 2]];

            const tri_bounds = computeTriangleBounds(v0, v1, v2);
            try octree.insert(prim_id, tri_bounds);
            prim_id += 1;
        }
    }
    std.log.info("Inserted {d} triangles into octree", .{prim_id});

    // Collect all octree node AABBs for visualization
    var node_bounds: std.ArrayListUnmanaged(Aabb) = .empty;
    defer node_bounds.deinit(allocator);
    try collectOctreeNodes(&octree.root, &node_bounds, allocator);
    std.log.info("Octree has {d} nodes", .{node_bounds.items.len});

    // Initialize debug visualizer
    const win_size = window.getSize();
    const aspect = @as(f32, @floatFromInt(win_size[0])) / @as(f32, @floatFromInt(win_size[1]));

    var visualizer = try DebugVisualizer.init(allocator, aspect);
    defer visualizer.deinit();

    // Upload model to GPU
    try visualizer.uploadModel(&model);

    // Set up camera to view the model
    const center = model_bounds.center();
    const size = model_bounds.bmax.toArray()[1] - model_bounds.bmin.toArray()[1];
    const camera_distance = size * 2.5;

    // Track time for animation
    var timer = try std.time.Timer.start();

    // Main loop
    while (!window.shouldClose()) {
        zglfw.pollEvents();

        const elapsed = @as(f32, @floatFromInt(timer.read())) / 1_000_000_000.0;

        // Get window/framebuffer sizes
        const current_win_size = window.getSize();
        const fb_size = window.getFramebufferSize();
        const fb_width: u32 = @intCast(fb_size[0]);
        const fb_height: u32 = @intCast(fb_size[1]);

        // Update camera aspect ratio and position (orbit around model)
        const current_aspect = @as(f32, @floatFromInt(current_win_size[0])) / @as(f32, @floatFromInt(current_win_size[1]));
        visualizer.camera.setProjection(45.0, current_aspect, 0.1, 1000.0);

        const rotation_speed: f32 = 20.0;
        const angle = elapsed * rotation_speed * std.math.pi / 180.0;
        const eye_x = center.toArray()[0] + camera_distance * @cos(angle);
        const eye_z = center.toArray()[2] + camera_distance * @sin(angle);
        visualizer.camera.lookAt(
            ZaVec3.new(eye_x, center.toArray()[1] + size * 0.5, eye_z),
            ZaVec3.new(center.toArray()[0], center.toArray()[1], center.toArray()[2]),
            ZaVec3.new(0.0, 1.0, 0.0),
        );

        // Render
        visualizer.beginFrame(fb_width, fb_height);

        // Draw the model
        visualizer.drawModelWithTransform(&model, ZaMat4.identity());

        // Draw octree node wireframes
        for (node_bounds.items) |bounds| {
            const min_arr = bounds.bmin.toArray();
            const max_arr = bounds.bmax.toArray();
            visualizer.drawAabbWireframe(
                ZaVec3.new(min_arr[0], min_arr[1], min_arr[2]),
                ZaVec3.new(max_arr[0], max_arr[1], max_arr[2]),
                ZaVec3.new(0.2, 0.8, 0.2), // green wireframes
            );
        }

        window.swapBuffers();
    }
}

fn loadBunnyModel(allocator: std.mem.Allocator) !Model {
    const ObjLoader = chad.objloader.ObjLoader;

    // Try to load from resources directory
    const paths = [_][]const u8{
        "resources/bunny.obj",
        "../resources/bunny.obj",
        "../../resources/bunny.obj",
    };

    for (paths) |path| {
        if (std.fs.cwd().access(path, .{})) |_| {
            return try ObjLoader.loadModel(allocator, path, 20.0);
        } else |_| {
            continue;
        }
    }

    return error.BunnyNotFound;
}

fn computeModelBounds(model: *const Model) Aabb {
    var min = Vec3.fromArray(&.{ std.math.inf(f32), std.math.inf(f32), std.math.inf(f32) });
    var max = Vec3.fromArray(&.{ -std.math.inf(f32), -std.math.inf(f32), -std.math.inf(f32) });

    for (model.meshes.items) |*mesh| {
        for (mesh.vertices) |v| {
            const min_arr = min.toArray();
            const max_arr = max.toArray();
            min = Vec3.fromArray(&.{
                @min(min_arr[0], v[0]),
                @min(min_arr[1], v[1]),
                @min(min_arr[2], v[2]),
            });
            max = Vec3.fromArray(&.{
                @max(max_arr[0], v[0]),
                @max(max_arr[1], v[1]),
                @max(max_arr[2], v[2]),
            });
        }
    }

    return .{ .bmin = min, .bmax = max };
}

fn computeTriangleBounds(v0: [3]f32, v1: [3]f32, v2: [3]f32) Aabb {
    return .{
        .bmin = Vec3.fromArray(&.{
            @min(@min(v0[0], v1[0]), v2[0]),
            @min(@min(v0[1], v1[1]), v2[1]),
            @min(@min(v0[2], v1[2]), v2[2]),
        }),
        .bmax = Vec3.fromArray(&.{
            @max(@max(v0[0], v1[0]), v2[0]),
            @max(@max(v0[1], v1[1]), v2[1]),
            @max(@max(v0[2], v1[2]), v2[2]),
        }),
    };
}

fn collectOctreeNodes(node: *const Octree.Node, results: *std.ArrayListUnmanaged(Aabb), allocator: std.mem.Allocator) !void {
    try results.append(allocator, node.bounds);

    if (node.children) |children| {
        for (children) |*child| {
            try collectOctreeNodes(child, results, allocator);
        }
    }
}
