const std = @import("std");
const geometry = @import("geometry.zig");
const ObjLoader = @import("objloader.zig").ObjLoader;
const Octree = @import("octree.zig").Octree;
const Ray = @import("octree.zig").Ray;

const Vec3 = geometry.Vec3;
const Aabb = geometry.Aabb;
const PrimId = geometry.PrimId;

/// Compute AABB for a triangle given vertex indices
fn triangleAabb(vertices: []const [3]f32, idx0: u32, idx1: u32, idx2: u32) Aabb {
    const v0 = vertices[idx0];
    const v1 = vertices[idx1];
    const v2 = vertices[idx2];

    return .{
        .bmin = Vec3.fromArray(&.{
            @min(v0[0], @min(v1[0], v2[0])),
            @min(v0[1], @min(v1[1], v2[1])),
            @min(v0[2], @min(v1[2], v2[2])),
        }),
        .bmax = Vec3.fromArray(&.{
            @max(v0[0], @max(v1[0], v2[0])),
            @max(v0[1], @max(v1[1], v2[1])),
            @max(v0[2], @max(v1[2], v2[2])),
        }),
    };
}

/// Compute AABB for entire mesh
fn meshAabb(vertices: []const [3]f32) Aabb {
    var bmin = [3]f32{ std.math.inf(f32), std.math.inf(f32), std.math.inf(f32) };
    var bmax = [3]f32{ -std.math.inf(f32), -std.math.inf(f32), -std.math.inf(f32) };

    for (vertices) |v| {
        bmin[0] = @min(bmin[0], v[0]);
        bmin[1] = @min(bmin[1], v[1]);
        bmin[2] = @min(bmin[2], v[2]);
        bmax[0] = @max(bmax[0], v[0]);
        bmax[1] = @max(bmax[1], v[1]);
        bmax[2] = @max(bmax[2], v[2]);
    }

    return .{
        .bmin = Vec3.fromArray(&bmin),
        .bmax = Vec3.fromArray(&bmax),
    };
}

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    // Load the bunny model
    var model = try ObjLoader.loadModel(allocator, "resources/bunny.obj", null);
    defer model.deinit();

    std.debug.print("Loaded bunny with {d} vertices and {d} triangles\n", .{
        model.totalVertexCount(),
        model.totalTriangleCount(),
    });

    // Build octree from triangles
    const mesh = model.meshes.items[0];
    const bounds = meshAabb(mesh.vertices);

    const bmin = bounds.bmin.toArray();
    const bmax = bounds.bmax.toArray();
    std.debug.print("Mesh bounds: ({d:.4}, {d:.4}, {d:.4}) - ({d:.4}, {d:.4}, {d:.4})\n", .{
        bmin[0], bmin[1], bmin[2],
        bmax[0], bmax[1], bmax[2],
    });

    var octree = Octree.init(allocator, bounds, .{});
    defer octree.deinit();

    // Insert all triangles
    const tri_count = mesh.indices.len / 3;
    for (0..tri_count) |i| {
        const base = i * 3;
        const aabb = triangleAabb(
            mesh.vertices,
            mesh.indices[base],
            mesh.indices[base + 1],
            mesh.indices[base + 2],
        );
        try octree.insert(@intCast(i), aabb);
    }
    std.debug.print("Inserted {d} triangles into octree\n", .{tri_count});

    // Test ray intersection (ray through center of mesh)
    const center_x = (bmin[0] + bmax[0]) * 0.5;
    const center_y = (bmin[1] + bmax[1]) * 0.5;
    const center_z = (bmin[2] + bmax[2]) * 0.5;

    var ray_results: std.ArrayListUnmanaged(PrimId) = .empty;
    defer ray_results.deinit(allocator);

    try octree.queryRayIntersection(.{
        .origin = Vec3.fromArray(&.{ center_x - 1.0, center_y, center_z }),
        .direction = Vec3.fromArray(&.{ 1, 0, 0 }),
    }, &ray_results);

    std.debug.print("Ray through center hit {d} triangle AABBs\n", .{ray_results.items.len});

    // Test AABB overlap (small box at center)
    var aabb_results: std.ArrayListUnmanaged(PrimId) = .empty;
    defer aabb_results.deinit(allocator);

    const query_size: f32 = 0.01;
    try octree.queryAabbOverlap(.{
        .bmin = Vec3.fromArray(&.{ center_x - query_size, center_y - query_size, center_z - query_size }),
        .bmax = Vec3.fromArray(&.{ center_x + query_size, center_y + query_size, center_z + query_size }),
    }, &aabb_results);

    std.debug.print("AABB query at center found {d} overlapping triangles\n", .{aabb_results.items.len});
}

// Tests

test "octree with bunny - ray intersection" {
    const allocator = std.testing.allocator;

    var model = try ObjLoader.loadModel(allocator, "resources/bunny.obj", null);
    defer model.deinit();

    const mesh = model.meshes.items[0];
    const bounds = meshAabb(mesh.vertices);

    var octree = Octree.init(allocator, bounds, .{});
    defer octree.deinit();

    // Insert all triangles
    const tri_count = mesh.indices.len / 3;
    for (0..tri_count) |i| {
        const base = i * 3;
        const aabb = triangleAabb(
            mesh.vertices,
            mesh.indices[base],
            mesh.indices[base + 1],
            mesh.indices[base + 2],
        );
        try octree.insert(@intCast(i), aabb);
    }

    // Ray through center should hit something
    const bmin = bounds.bmin.toArray();
    const bmax = bounds.bmax.toArray();
    const center_y = (bmin[1] + bmax[1]) * 0.5;
    const center_z = (bmin[2] + bmax[2]) * 0.5;

    var results: std.ArrayListUnmanaged(PrimId) = .empty;
    defer results.deinit(allocator);

    try octree.queryRayIntersection(.{
        .origin = Vec3.fromArray(&.{ bmin[0] - 1.0, center_y, center_z }),
        .direction = Vec3.fromArray(&.{ 1, 0, 0 }),
    }, &results);

    try std.testing.expect(results.items.len > 0);
}

test "octree with bunny - aabb overlap" {
    const allocator = std.testing.allocator;

    var model = try ObjLoader.loadModel(allocator, "resources/bunny.obj", null);
    defer model.deinit();

    const mesh = model.meshes.items[0];
    const bounds = meshAabb(mesh.vertices);

    var octree = Octree.init(allocator, bounds, .{});
    defer octree.deinit();

    // Insert all triangles
    const tri_count = mesh.indices.len / 3;
    for (0..tri_count) |i| {
        const base = i * 3;
        const aabb = triangleAabb(
            mesh.vertices,
            mesh.indices[base],
            mesh.indices[base + 1],
            mesh.indices[base + 2],
        );
        try octree.insert(@intCast(i), aabb);
    }

    // Query at center should find triangles
    const bmin = bounds.bmin.toArray();
    const bmax = bounds.bmax.toArray();
    const center_x = (bmin[0] + bmax[0]) * 0.5;
    const center_y = (bmin[1] + bmax[1]) * 0.5;
    const center_z = (bmin[2] + bmax[2]) * 0.5;

    var results: std.ArrayListUnmanaged(PrimId) = .empty;
    defer results.deinit(allocator);

    const query_size: f32 = 0.01;
    try octree.queryAabbOverlap(.{
        .bmin = Vec3.fromArray(&.{ center_x - query_size, center_y - query_size, center_z - query_size }),
        .bmax = Vec3.fromArray(&.{ center_x + query_size, center_y + query_size, center_z + query_size }),
    }, &results);

    try std.testing.expect(results.items.len > 0);
}

test "octree with bunny - ray miss" {
    const allocator = std.testing.allocator;

    var model = try ObjLoader.loadModel(allocator, "resources/bunny.obj", null);
    defer model.deinit();

    const mesh = model.meshes.items[0];
    const bounds = meshAabb(mesh.vertices);

    var octree = Octree.init(allocator, bounds, .{});
    defer octree.deinit();

    // Insert all triangles
    const tri_count = mesh.indices.len / 3;
    for (0..tri_count) |i| {
        const base = i * 3;
        const aabb = triangleAabb(
            mesh.vertices,
            mesh.indices[base],
            mesh.indices[base + 1],
            mesh.indices[base + 2],
        );
        try octree.insert(@intCast(i), aabb);
    }

    // Ray far outside should miss
    const bmax = bounds.bmax.toArray();

    var results: std.ArrayListUnmanaged(PrimId) = .empty;
    defer results.deinit(allocator);

    try octree.queryRayIntersection(.{
        .origin = Vec3.fromArray(&.{ bmax[0] + 10.0, bmax[1] + 10.0, bmax[2] + 10.0 }),
        .direction = Vec3.fromArray(&.{ 1, 0, 0 }),
    }, &results);

    try std.testing.expectEqual(@as(usize, 0), results.items.len);
}

test "octree with bunny - empty aabb query" {
    const allocator = std.testing.allocator;

    var model = try ObjLoader.loadModel(allocator, "resources/bunny.obj", null);
    defer model.deinit();

    const mesh = model.meshes.items[0];
    const bounds = meshAabb(mesh.vertices);

    var octree = Octree.init(allocator, bounds, .{});
    defer octree.deinit();

    // Insert all triangles
    const tri_count = mesh.indices.len / 3;
    for (0..tri_count) |i| {
        const base = i * 3;
        const aabb = triangleAabb(
            mesh.vertices,
            mesh.indices[base],
            mesh.indices[base + 1],
            mesh.indices[base + 2],
        );
        try octree.insert(@intCast(i), aabb);
    }

    // Query far outside should find nothing
    const bmax = bounds.bmax.toArray();

    var results: std.ArrayListUnmanaged(PrimId) = .empty;
    defer results.deinit(allocator);

    try octree.queryAabbOverlap(.{
        .bmin = Vec3.fromArray(&.{ bmax[0] + 10.0, bmax[1] + 10.0, bmax[2] + 10.0 }),
        .bmax = Vec3.fromArray(&.{ bmax[0] + 11.0, bmax[1] + 11.0, bmax[2] + 11.0 }),
    }, &results);

    try std.testing.expectEqual(@as(usize, 0), results.items.len);
}
