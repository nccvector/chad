const std = @import("std");
const lmao = @import("lmao");

pub const Vec3 = lmao.Vec3f;
pub const Mat4 = lmao.Mat4f;

pub const PrimId = u32;

/// Unique identifier for a mesh
pub const MeshId = u32;

pub const Aabb = struct {
    bmin: Vec3,
    bmax: Vec3,
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
