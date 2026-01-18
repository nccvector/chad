const std = @import("std");
const lmao = @import("lmao");

pub const Vec3 = lmao.Vec3f;
pub const Mat4 = lmao.Mat4f;

pub const PrimId = u32;

/// Unique identifier for a mesh
pub const MeshId = u32;

pub const Ray = struct {
    origin: Vec3,
    direction: Vec3,

    /// Creates a ray from origin to target point
    pub fn fromPoints(origin: Vec3, target: Vec3) Ray {
        return .{
            .origin = origin,
            .direction = target.sub(origin).normalized(),
        };
    }

    /// Returns the point along the ray at parameter t: origin + t * direction
    pub fn at(self: Ray, t: f32) Vec3 {
        return .{ .data = self.origin.data + @as(@Vector(3, f32), @splat(t)) * self.direction.data };
    }
};

/// Result of a ray-triangle intersection test
pub const RayHit = struct {
    t: f32, // Distance along ray
    u: f32, // Barycentric coordinate u
    v: f32, // Barycentric coordinate v
    // w = 1 - u - v (barycentric coordinate)

    /// Returns the third barycentric coordinate w
    pub inline fn w(self: RayHit) f32 {
        return 1.0 - self.u - self.v;
    }
};

/// Triangle struct for intersection tests
pub const Triangle = struct {
    v0: Vec3,
    v1: Vec3,
    v2: Vec3,

    /// Creates a triangle from three vertices
    pub fn init(v0: Vec3, v1: Vec3, v2: Vec3) Triangle {
        return .{ .v0 = v0, .v1 = v1, .v2 = v2 };
    }

    /// Creates a triangle from a mesh at the given triangle index
    pub fn fromMesh(mesh: *const Mesh, tri_idx: usize) Triangle {
        const idx0 = mesh.indices[tri_idx * 3 + 0];
        const idx1 = mesh.indices[tri_idx * 3 + 1];
        const idx2 = mesh.indices[tri_idx * 3 + 2];
        return .{
            .v0 = Vec3.fromArray(&mesh.vertices[idx0]),
            .v1 = Vec3.fromArray(&mesh.vertices[idx1]),
            .v2 = Vec3.fromArray(&mesh.vertices[idx2]),
        };
    }

    /// Computes the geometric normal of the triangle (non-normalized)
    pub fn normal(self: Triangle) Vec3 {
        const edge1 = self.v1.sub(self.v0);
        const edge2 = self.v2.sub(self.v0);
        return edge1.cross(edge2);
    }

    /// Computes the normalized geometric normal of the triangle
    pub fn normalNormalized(self: Triangle) Vec3 {
        return self.normal().normalized();
    }

    /// Computes the AABB bounding box of the triangle
    pub fn bounds(self: Triangle) Aabb {
        return .{
            .bmin = self.v0.min(self.v1).min(self.v2),
            .bmax = self.v0.max(self.v1).max(self.v2),
        };
    }

    /// Computes the centroid of the triangle
    pub fn centroid(self: Triangle) Vec3 {
        const third: @Vector(3, f32) = @splat(1.0 / 3.0);
        return .{ .data = (self.v0.data + self.v1.data + self.v2.data) * third };
    }

    /// Möller-Trumbore ray-triangle intersection algorithm.
    /// Returns the hit information if the ray intersects the triangle, null otherwise.
    /// The `backface_cull` parameter controls whether to reject back-facing triangles.
    pub fn intersectRay(self: Triangle, ray: Ray, backface_cull: bool) ?RayHit {
        const epsilon: f32 = 1e-8;

        const edge1 = self.v1.sub(self.v0);
        const edge2 = self.v2.sub(self.v0);

        const h = ray.direction.cross(edge2);
        const a = edge1.dotProduct(h);

        // Check if ray is parallel to triangle
        if (backface_cull) {
            // Backface culling: reject if a < epsilon (back-facing or parallel)
            if (a < epsilon) return null;
        } else {
            // No culling: reject only if truly parallel
            if (@abs(a) < epsilon) return null;
        }

        const f = 1.0 / a;
        const s = ray.origin.sub(self.v0);
        const u = f * s.dotProduct(h);

        if (u < 0.0 or u > 1.0) return null;

        const q = s.cross(edge1);
        const v = f * ray.direction.dotProduct(q);

        if (v < 0.0 or u + v > 1.0) return null;

        const t = f * edge2.dotProduct(q);

        if (t > epsilon) {
            return .{ .t = t, .u = u, .v = v };
        }

        return null; // Line intersection but not ray (t <= 0)
    }

    /// Ray-triangle intersection that also considers negative t values (behind ray origin).
    /// Useful for bidirectional ray casting.
    pub fn intersectRayBidirectional(self: Triangle, ray: Ray) ?RayHit {
        const epsilon: f32 = 1e-8;

        const edge1 = self.v1.sub(self.v0);
        const edge2 = self.v2.sub(self.v0);

        const h = ray.direction.cross(edge2);
        const a = edge1.dotProduct(h);

        if (@abs(a) < epsilon) return null;

        const f = 1.0 / a;
        const s = ray.origin.sub(self.v0);
        const u = f * s.dotProduct(h);

        if (u < 0.0 or u > 1.0) return null;

        const q = s.cross(edge1);
        const v = f * ray.direction.dotProduct(q);

        if (v < 0.0 or u + v > 1.0) return null;

        const t = f * edge2.dotProduct(q);

        // Accept any t value (positive or negative)
        if (@abs(t) > epsilon) {
            return .{ .t = t, .u = u, .v = v };
        }

        return null;
    }
};

pub const Aabb = struct {
    pub const Vec = @Vector(3, f32);

    bmin: Vec3,
    bmax: Vec3,

    pub inline fn center(self: Aabb) Vec3 {
        const half: Vec = @splat(0.5);
        const min_v: Vec = self.bmin.data;
        const max_v: Vec = self.bmax.data;
        const result = (min_v + max_v) * half;
        return .{ .data = result };
    }

    /// Overlap test (closed: touching counts as overlap)
    pub inline fn overlaps(self: Aabb, other: Aabb) bool {
        const a_min: Vec = self.bmin.data;
        const a_max: Vec = self.bmax.data;
        const b_min: Vec = other.bmin.data;
        const b_max: Vec = other.bmax.data;

        const lo_ok: @Vector(3, bool) = a_min <= b_max;
        const hi_ok: @Vector(3, bool) = a_max >= b_min;
        return @reduce(.And, lo_ok) and @reduce(.And, hi_ok);
    }

    /// Strict overlap test (touching does NOT count)
    pub inline fn overlapsStrict(self: Aabb, other: Aabb) bool {
        const a_min: Vec = self.bmin.data;
        const a_max: Vec = self.bmax.data;
        const b_min: Vec = other.bmin.data;
        const b_max: Vec = other.bmax.data;

        const lo_ok: @Vector(3, bool) = a_min < b_max;
        const hi_ok: @Vector(3, bool) = a_max > b_min;
        return @reduce(.And, lo_ok) and @reduce(.And, hi_ok);
    }

    /// Ray-AABB intersection test
    pub inline fn intersectsRay(self: Aabb, ray: Ray) bool {
        const origin: Vec = ray.origin.data;
        const dir: Vec = ray.direction.data;
        const box_min: Vec = self.bmin.data;
        const box_max: Vec = self.bmax.data;

        const epsilon: Vec = @splat(1e-8);
        const abs_dir = @abs(dir);
        const is_parallel = abs_dir < epsilon;

        // Check if ray is parallel and outside slab
        if (@reduce(.Or, is_parallel)) {
            const origin_arr = ray.origin.toArray();
            const dir_arr = ray.direction.toArray();
            const bmin_arr = self.bmin.toArray();
            const bmax_arr = self.bmax.toArray();
            inline for (0..3) |i| {
                if (@abs(dir_arr[i]) < 1e-8) {
                    if (origin_arr[i] < bmin_arr[i] or origin_arr[i] > bmax_arr[i]) {
                        return false;
                    }
                }
            }
        }

        const one: Vec = @splat(1.0);
        const safe_dir = @select(f32, is_parallel, one, dir);
        const inv_dir = one / safe_dir;

        const t1 = (box_min - origin) * inv_dir;
        const t2 = (box_max - origin) * inv_dir;

        var t_near = @min(t1, t2);
        var t_far = @max(t1, t2);

        // For parallel dimensions, use -inf/+inf so they don't affect the reduction
        const neg_inf: Vec = @splat(-std.math.inf(f32));
        const pos_inf: Vec = @splat(std.math.inf(f32));
        t_near = @select(f32, is_parallel, neg_inf, t_near);
        t_far = @select(f32, is_parallel, pos_inf, t_far);

        const t_min = @reduce(.Max, t_near);
        const t_max = @reduce(.Min, t_far);

        return t_max >= 0 and t_min <= t_max;
    }
};

/// A mesh containing only geometry data (no GPU resources)
pub const Mesh = struct {
    id: MeshId,
    vertices: []const [3]f32,
    indices: []const u32,
    allocator: std.mem.Allocator,

    /// Generate a unique mesh ID
    var next_id: MeshId = 0;

    fn generateId() MeshId {
        const id = next_id;
        next_id += 1;
        return id;
    }

    /// Create a mesh from vertex and index data (takes ownership)
    pub fn init(allocator: std.mem.Allocator, vertices: []const [3]f32, indices: []const u32) Mesh {
        return .{
            .id = generateId(),
            .vertices = vertices,
            .indices = indices,
            .allocator = allocator,
        };
    }

    /// Create a mesh by copying data
    pub fn initCopy(allocator: std.mem.Allocator, vertices: []const [3]f32, indices: []const u32) !Mesh {
        const verts = try allocator.dupe([3]f32, vertices);
        errdefer allocator.free(verts);
        const inds = try allocator.dupe(u32, indices);

        return .{
            .id = generateId(),
            .vertices = verts,
            .indices = inds,
            .allocator = allocator,
        };
    }

    pub fn deinit(self: *Mesh) void {
        self.allocator.free(self.vertices);
        self.allocator.free(self.indices);
        self.* = undefined;
    }

    pub fn vertexCount(self: *const Mesh) usize {
        return self.vertices.len;
    }

    pub fn triangleCount(self: *const Mesh) usize {
        return self.indices.len / 3;
    }
};

/// A model containing one or more meshes and a transform
pub const Model = struct {
    meshes: std.ArrayListUnmanaged(Mesh),
    transform: Mat4,
    allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) Model {
        return .{
            .meshes = .empty,
            .transform = Mat4.identity(),
            .allocator = allocator,
        };
    }

    pub fn deinit(self: *Model) void {
        for (self.meshes.items) |*mesh| {
            mesh.deinit();
        }
        self.meshes.deinit(self.allocator);
    }

    pub fn addMesh(self: *Model, mesh: Mesh) !void {
        try self.meshes.append(self.allocator, mesh);
    }

    pub fn totalVertexCount(self: *const Model) usize {
        var total: usize = 0;
        for (self.meshes.items) |*mesh| {
            total += mesh.vertexCount();
        }
        return total;
    }

    pub fn totalTriangleCount(self: *const Model) usize {
        var total: usize = 0;
        for (self.meshes.items) |*mesh| {
            total += mesh.triangleCount();
        }
        return total;
    }
};

// Tests
test "Mesh creation" {
    const allocator = std.testing.allocator;

    const vertices = [_][3]f32{
        .{ 0.0, 0.0, 0.0 },
        .{ 1.0, 0.0, 0.0 },
        .{ 0.0, 1.0, 0.0 },
    };
    const indices = [_]u32{ 0, 1, 2 };

    var mesh = try Mesh.initCopy(allocator, &vertices, &indices);
    defer mesh.deinit();

    try std.testing.expectEqual(@as(usize, 3), mesh.vertexCount());
    try std.testing.expectEqual(@as(usize, 1), mesh.triangleCount());
}

test "Model creation" {
    const allocator = std.testing.allocator;

    var model = Model.init(allocator);
    defer model.deinit();

    const vertices = [_][3]f32{
        .{ 0.0, 0.0, 0.0 },
        .{ 1.0, 0.0, 0.0 },
        .{ 0.0, 1.0, 0.0 },
    };
    const indices = [_]u32{ 0, 1, 2 };

    const mesh = try Mesh.initCopy(allocator, &vertices, &indices);
    try model.addMesh(mesh);

    try std.testing.expectEqual(@as(usize, 3), model.totalVertexCount());
    try std.testing.expectEqual(@as(usize, 1), model.totalTriangleCount());
}
