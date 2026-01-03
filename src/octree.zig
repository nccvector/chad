const std = @import("std");
const geometry = @import("geometry.zig");

const Vec3 = geometry.Vec3;
const Aabb = geometry.Aabb;
const PrimId = geometry.PrimId;

pub const Ray = struct {
    origin: Vec3,
    direction: Vec3,
};

pub const Octree = struct {
    const Self = @This();

    pub const Config = struct {
        max_depth: u8 = 8,
        max_primitives_per_node: usize = 8,
    };

    const Node = struct {
        bounds: Aabb,
        children: ?*[8]Node = null,
        primitives: std.ArrayListUnmanaged(Entry) = .empty,
        depth: u8 = 0,

        const Entry = struct {
            id: PrimId,
            bounds: Aabb,
        };

        fn deinit(self: *Node, allocator: std.mem.Allocator) void {
            self.primitives.deinit(allocator);
            if (self.children) |children| {
                for (children) |*child| {
                    child.deinit(allocator);
                }
                allocator.destroy(children);
            }
        }

        fn isLeaf(self: *const Node) bool {
            return self.children == null;
        }
    };

    root: Node,
    config: Config,
    allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator, bounds: Aabb, config: Config) Self {
        return .{
            .root = .{
                .bounds = bounds,
                .depth = 0,
            },
            .config = config,
            .allocator = allocator,
        };
    }

    pub fn deinit(self: *Self) void {
        self.root.deinit(self.allocator);
    }

    /// Insert a primitive with its AABB into the octree
    pub fn insert(self: *Self, id: PrimId, bounds: Aabb) !void {
        try self.insertIntoNode(&self.root, id, bounds);
    }

    fn insertIntoNode(self: *Self, node: *Node, id: PrimId, bounds: Aabb) !void {
        // If primitive doesn't overlap this node, skip
        if (!aabbOverlap(node.bounds, bounds)) return;

        if (node.isLeaf()) {
            // Add to this leaf node
            try node.primitives.append(self.allocator, .{ .id = id, .bounds = bounds });

            // Split if needed
            if (node.primitives.items.len > self.config.max_primitives_per_node and
                node.depth < self.config.max_depth)
            {
                try self.splitNode(node);
            }
        } else {
            // Insert into children
            for (node.children.?) |*child| {
                try self.insertIntoNode(child, id, bounds);
            }
        }
    }

    fn splitNode(self: *Self, node: *Node) !void {
        const children = try self.allocator.create([8]Node);

        const center = aabbCenter(node.bounds);
        const min = vec3ToArray(node.bounds.bmin);
        const max = vec3ToArray(node.bounds.bmax);
        const c = vec3ToArray(center);

        // Create 8 child nodes with subdivided bounds
        inline for (0..8) |i| {
            const x_min = if (i & 1 == 0) min[0] else c[0];
            const x_max = if (i & 1 == 0) c[0] else max[0];
            const y_min = if (i & 2 == 0) min[1] else c[1];
            const y_max = if (i & 2 == 0) c[1] else max[1];
            const z_min = if (i & 4 == 0) min[2] else c[2];
            const z_max = if (i & 4 == 0) c[2] else max[2];

            children[i] = .{
                .bounds = .{
                    .bmin = Vec3.fromArray(&.{ x_min, y_min, z_min }),
                    .bmax = Vec3.fromArray(&.{ x_max, y_max, z_max }),
                },
                .depth = node.depth + 1,
            };
        }

        node.children = children;

        // Re-insert primitives into children
        for (node.primitives.items) |entry| {
            for (children) |*child| {
                if (aabbOverlap(child.bounds, entry.bounds)) {
                    try child.primitives.append(self.allocator, entry);
                }
            }
        }

        // Clear parent primitives
        node.primitives.clearAndFree(self.allocator);
    }

    /// Query all primitives whose AABBs overlap with the given AABB
    pub fn queryAabbOverlap(self: *const Self, bounds: Aabb, results: *std.ArrayListUnmanaged(PrimId)) !void {
        try self.queryAabbNode(&self.root, bounds, results);
    }

    fn queryAabbNode(self: *const Self, node: *const Node, bounds: Aabb, results: *std.ArrayListUnmanaged(PrimId)) !void {
        if (!aabbOverlap(node.bounds, bounds)) return;

        // Check primitives at this node
        for (node.primitives.items) |entry| {
            if (aabbOverlap(entry.bounds, bounds)) {
                // Avoid duplicates
                var found = false;
                for (results.items) |existing| {
                    if (existing == entry.id) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    try results.append(self.allocator, entry.id);
                }
            }
        }

        // Recurse into children
        if (node.children) |children| {
            for (children) |*child| {
                try self.queryAabbNode(child, bounds, results);
            }
        }
    }

    /// Query all primitives whose AABBs intersect with the given ray
    pub fn queryRayIntersection(self: *const Self, ray: Ray, results: *std.ArrayListUnmanaged(PrimId)) !void {
        try self.queryRayNode(&self.root, ray, results);
    }

    fn queryRayNode(self: *const Self, node: *const Node, ray: Ray, results: *std.ArrayListUnmanaged(PrimId)) !void {
        if (!rayAabbIntersect(ray, node.bounds)) return;

        // Check primitives at this node
        for (node.primitives.items) |entry| {
            if (rayAabbIntersect(ray, entry.bounds)) {
                // Avoid duplicates
                var found = false;
                for (results.items) |existing| {
                    if (existing == entry.id) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    try results.append(self.allocator, entry.id);
                }
            }
        }

        // Recurse into children
        if (node.children) |children| {
            for (children) |*child| {
                try self.queryRayNode(child, ray, results);
            }
        }
    }

    /// Clear all primitives from the octree
    pub fn clear(self: *Self) void {
        self.root.deinit(self.allocator);
        self.root = .{
            .bounds = self.root.bounds,
            .depth = 0,
        };
    }
};

// Helper functions

fn vec3ToArray(v: Vec3) [3]f32 {
    return v.toArray()[0..3].*;
}

fn aabbCenter(aabb: Aabb) Vec3 {
    const min = vec3ToArray(aabb.bmin);
    const max = vec3ToArray(aabb.bmax);
    return Vec3.fromArray(&.{
        (min[0] + max[0]) * 0.5,
        (min[1] + max[1]) * 0.5,
        (min[2] + max[2]) * 0.5,
    });
}

fn aabbOverlap(a: Aabb, b: Aabb) bool {
    const a_min = vec3ToArray(a.bmin);
    const a_max = vec3ToArray(a.bmax);
    const b_min = vec3ToArray(b.bmin);
    const b_max = vec3ToArray(b.bmax);

    return a_min[0] <= b_max[0] and a_max[0] >= b_min[0] and
        a_min[1] <= b_max[1] and a_max[1] >= b_min[1] and
        a_min[2] <= b_max[2] and a_max[2] >= b_min[2];
}

fn rayAabbIntersect(ray: Ray, aabb: Aabb) bool {
    const origin = vec3ToArray(ray.origin);
    const dir = vec3ToArray(ray.direction);
    const box_min = vec3ToArray(aabb.bmin);
    const box_max = vec3ToArray(aabb.bmax);

    var t_min: f32 = -std.math.inf(f32);
    var t_max: f32 = std.math.inf(f32);

    inline for (0..3) |i| {
        if (@abs(dir[i]) < 1e-8) {
            // Ray parallel to slab
            if (origin[i] < box_min[i] or origin[i] > box_max[i]) {
                return false;
            }
        } else {
            const inv_d = 1.0 / dir[i];
            var t1 = (box_min[i] - origin[i]) * inv_d;
            var t2 = (box_max[i] - origin[i]) * inv_d;

            if (t1 > t2) {
                const tmp = t1;
                t1 = t2;
                t2 = tmp;
            }

            t_min = @max(t_min, t1);
            t_max = @min(t_max, t2);

            if (t_min > t_max) {
                return false;
            }
        }
    }

    // Return true if intersection is in front of ray origin
    return t_max >= 0;
}

// Tests
test "octree basic insert and query" {
    const allocator = std.testing.allocator;

    var octree = Octree.init(allocator, .{
        .bmin = Vec3.fromArray(&.{ -10, -10, -10 }),
        .bmax = Vec3.fromArray(&.{ 10, 10, 10 }),
    }, .{});
    defer octree.deinit();

    // Insert some primitives
    try octree.insert(0, .{
        .bmin = Vec3.fromArray(&.{ -1, -1, -1 }),
        .bmax = Vec3.fromArray(&.{ 1, 1, 1 }),
    });
    try octree.insert(1, .{
        .bmin = Vec3.fromArray(&.{ 2, 2, 2 }),
        .bmax = Vec3.fromArray(&.{ 4, 4, 4 }),
    });
    try octree.insert(2, .{
        .bmin = Vec3.fromArray(&.{ -5, -5, -5 }),
        .bmax = Vec3.fromArray(&.{ -3, -3, -3 }),
    });

    // Query AABB overlap
    var results: std.ArrayListUnmanaged(PrimId) = .empty;
    defer results.deinit(allocator);

    try octree.queryAabbOverlap(.{
        .bmin = Vec3.fromArray(&.{ 0, 0, 0 }),
        .bmax = Vec3.fromArray(&.{ 3, 3, 3 }),
    }, &results);

    try std.testing.expectEqual(@as(usize, 2), results.items.len);
}

test "octree ray intersection" {
    const allocator = std.testing.allocator;

    var octree = Octree.init(allocator, .{
        .bmin = Vec3.fromArray(&.{ -10, -10, -10 }),
        .bmax = Vec3.fromArray(&.{ 10, 10, 10 }),
    }, .{});
    defer octree.deinit();

    try octree.insert(0, .{
        .bmin = Vec3.fromArray(&.{ -1, -1, -1 }),
        .bmax = Vec3.fromArray(&.{ 1, 1, 1 }),
    });
    try octree.insert(1, .{
        .bmin = Vec3.fromArray(&.{ 5, -1, -1 }),
        .bmax = Vec3.fromArray(&.{ 7, 1, 1 }),
    });

    var results: std.ArrayListUnmanaged(PrimId) = .empty;
    defer results.deinit(allocator);

    // Ray along X axis should hit both
    try octree.queryRayIntersection(.{
        .origin = Vec3.fromArray(&.{ -10, 0, 0 }),
        .direction = Vec3.fromArray(&.{ 1, 0, 0 }),
    }, &results);

    try std.testing.expectEqual(@as(usize, 2), results.items.len);
}
