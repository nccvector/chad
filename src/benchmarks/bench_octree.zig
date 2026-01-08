const std = @import("std");
const config = @import("config");
const geometry = @import("geometry");
const octree_mod = @import("octree");

const enable_diagnostics = config.enable_diagnostics;

const Vec3 = geometry.Vec3;
const Aabb = geometry.Aabb;
const Ray = octree_mod.Ray;
const Octree = octree_mod.Octree;
const PrimId = geometry.PrimId;

const doNotOptimizeAway = std.mem.doNotOptimizeAway;

fn print(comptime fmt: []const u8, args: anytype) void {
    const stdout = std.fs.File.stdout();
    var buf: [4096]u8 = undefined;
    const slice = std.fmt.bufPrint(&buf, fmt, args) catch return;
    _ = stdout.write(slice) catch {};
}

fn formatTime(ns: f64, buf: []u8) []const u8 {
    if (ns < 1_000) {
        return std.fmt.bufPrint(buf, "{d:.2}ns", .{ns}) catch "?";
    } else if (ns < 1_000_000) {
        return std.fmt.bufPrint(buf, "{d:.2}us", .{ns / 1_000.0}) catch "?";
    } else if (ns < 1_000_000_000) {
        return std.fmt.bufPrint(buf, "{d:.2}ms", .{ns / 1_000_000.0}) catch "?";
    } else {
        return std.fmt.bufPrint(buf, "{d:.2}s", .{ns / 1_000_000_000.0}) catch "?";
    }
}

fn randomFloat(rng: std.Random) f32 {
    return rng.float(f32) * 20.0 - 10.0;
}

fn randomAabb(rng: std.Random) Aabb {
    const x1 = randomFloat(rng);
    const y1 = randomFloat(rng);
    const z1 = randomFloat(rng);
    const x2 = x1 + rng.float(f32) * 2.0 + 0.1;
    const y2 = y1 + rng.float(f32) * 2.0 + 0.1;
    const z2 = z1 + rng.float(f32) * 2.0 + 0.1;
    return .{
        .bmin = Vec3.fromArray(&.{ x1, y1, z1 }),
        .bmax = Vec3.fromArray(&.{ x2, y2, z2 }),
    };
}

fn randomRay(rng: std.Random) Ray {
    const ox = randomFloat(rng);
    const oy = randomFloat(rng);
    const oz = randomFloat(rng);
    var dx = randomFloat(rng);
    var dy = randomFloat(rng);
    var dz = randomFloat(rng);
    const len = @sqrt(dx * dx + dy * dy + dz * dz);
    if (len > 0.001) {
        dx /= len;
        dy /= len;
        dz /= len;
    } else {
        dx = 1.0;
        dy = 0.0;
        dz = 0.0;
    }
    return .{
        .origin = Vec3.fromArray(&.{ ox, oy, oz }),
        .direction = Vec3.fromArray(&.{ dx, dy, dz }),
    };
}

fn randomSmallAabb(rng: std.Random) Aabb {
    // Small AABBs concentrated near origin to maximize node splitting
    const x1 = rng.float(f32) * 4.0 - 2.0;
    const y1 = rng.float(f32) * 4.0 - 2.0;
    const z1 = rng.float(f32) * 4.0 - 2.0;
    const size = rng.float(f32) * 0.2 + 0.05;
    return .{
        .bmin = Vec3.fromArray(&.{ x1, y1, z1 }),
        .bmax = Vec3.fromArray(&.{ x1 + size, y1 + size, z1 + size }),
    };
}

fn benchmarkSplitNode(num_splits: usize, primitives_per_split: usize, rng: std.Random, alloc: std.mem.Allocator) !f64 {
    // Directly benchmark splitNode by manually populating nodes and calling splitNode
    const world_bounds = Aabb{
        .bmin = Vec3.fromArray(&.{ -10, -10, -10 }),
        .bmax = Vec3.fromArray(&.{ 10, 10, 10 }),
    };

    // Pre-generate AABBs for all splits
    const total_primitives = num_splits * primitives_per_split;
    const aabbs = try alloc.alloc(Aabb, total_primitives);
    defer alloc.free(aabbs);

    for (0..total_primitives) |i| {
        aabbs[i] = randomSmallAabb(rng);
    }

    var total_ns: i128 = 0;

    for (0..num_splits) |split_i| {
        // Create a fresh octree for each split (high threshold to prevent auto-splits)
        var octree = Octree.init(alloc, world_bounds, .{
            .max_primitives_per_node = 1_000_000,
            .max_depth = 8,
        });
        defer octree.deinit();

        // Manually populate root node with primitives (bypassing insert to avoid any overhead)
        const base = split_i * primitives_per_split;
        for (0..primitives_per_split) |i| {
            try octree.root.primitives.append(alloc, .{
                .id = @intCast(base + i),
                .bounds = aabbs[base + i],
            });
        }

        // Time the splitNode call directly
        const start = std.time.nanoTimestamp();
        try octree.splitNode(&octree.root);
        const end = std.time.nanoTimestamp();

        total_ns += (end - start);
    }

    return @as(f64, @floatFromInt(total_ns)) / @as(f64, @floatFromInt(num_splits));
}

fn countNodes(node: *const Octree.Node) usize {
    var count: usize = 1;
    if (node.children) |children| {
        for (children) |*child| {
            count += countNodes(child);
        }
    }
    return count;
}

fn countPrimitiveEntries(node: *const Octree.Node) usize {
    var count: usize = node.primitives.items.len;
    if (node.children) |children| {
        for (children) |*child| {
            count += countPrimitiveEntries(child);
        }
    }
    return count;
}

fn maxDepth(node: *const Octree.Node) u8 {
    var max: u8 = node.depth;
    if (node.children) |children| {
        for (children) |*child| {
            max = @max(max, maxDepth(child));
        }
    }
    return max;
}

const InsertStats = struct {
    time_ns: f64,
    nodes: ?usize = null,
    entries: ?usize = null,
    depth: ?u8 = null,
    num_primitives: usize,

    fn duplication(self: InsertStats) f64 {
        return @as(f64, @floatFromInt(self.entries.?)) / @as(f64, @floatFromInt(self.num_primitives));
    }
};

fn benchmarkInsert(num_primitives: usize, rng: std.Random, alloc: std.mem.Allocator) !InsertStats {
    const aabbs = try alloc.alloc(Aabb, num_primitives);
    defer alloc.free(aabbs);

    for (0..num_primitives) |i| {
        aabbs[i] = randomSmallAabb(rng);
    }

    const world_bounds = Aabb{
        .bmin = Vec3.fromArray(&.{ -15, -15, -15 }),
        .bmax = Vec3.fromArray(&.{ 15, 15, 15 }),
    };

    var octree = Octree.init(alloc, world_bounds, .{});
    defer octree.deinit();

    const start = std.time.nanoTimestamp();
    for (0..num_primitives) |i| {
        try octree.insert(@intCast(i), aabbs[i]);
    }
    const end = std.time.nanoTimestamp();

    var stats = InsertStats{
        .time_ns = @as(f64, @floatFromInt(end - start)) / @as(f64, @floatFromInt(num_primitives)),
        .num_primitives = num_primitives,
    };

    if (enable_diagnostics) {
        stats.nodes = countNodes(&octree.root);
        stats.entries = countPrimitiveEntries(&octree.root);
        stats.depth = maxDepth(&octree.root);
    }

    return stats;
}

fn benchmarkQueryAabb(num_primitives: usize, num_queries: usize, rng: std.Random, alloc: std.mem.Allocator) !f64 {
    const world_bounds = Aabb{
        .bmin = Vec3.fromArray(&.{ -15, -15, -15 }),
        .bmax = Vec3.fromArray(&.{ 15, 15, 15 }),
    };

    var octree = Octree.init(alloc, world_bounds, .{});
    defer octree.deinit();

    // Populate the octree
    for (0..num_primitives) |i| {
        try octree.insert(@intCast(i), randomAabb(rng));
    }

    // Generate query AABBs
    const query_aabbs = try alloc.alloc(Aabb, num_queries);
    defer alloc.free(query_aabbs);
    for (0..num_queries) |i| {
        query_aabbs[i] = randomAabb(rng);
    }

    var results: std.ArrayListUnmanaged(PrimId) = .empty;
    defer results.deinit(alloc);

    var acc: usize = 0;
    const start = std.time.nanoTimestamp();
    for (0..num_queries) |i| {
        results.clearRetainingCapacity();
        try octree.queryAabbOverlap(query_aabbs[i], &results);
        acc += results.items.len;
    }
    const end = std.time.nanoTimestamp();
    doNotOptimizeAway(acc);

    return @as(f64, @floatFromInt(end - start)) / @as(f64, @floatFromInt(num_queries));
}

fn benchmarkQueryRay(num_primitives: usize, num_queries: usize, rng: std.Random, alloc: std.mem.Allocator) !f64 {
    const world_bounds = Aabb{
        .bmin = Vec3.fromArray(&.{ -15, -15, -15 }),
        .bmax = Vec3.fromArray(&.{ 15, 15, 15 }),
    };

    var octree = Octree.init(alloc, world_bounds, .{});
    defer octree.deinit();

    // Populate the octree
    for (0..num_primitives) |i| {
        try octree.insert(@intCast(i), randomAabb(rng));
    }

    // Generate query rays
    const query_rays = try alloc.alloc(Ray, num_queries);
    defer alloc.free(query_rays);
    for (0..num_queries) |i| {
        query_rays[i] = randomRay(rng);
    }

    var results: std.ArrayListUnmanaged(PrimId) = .empty;
    defer results.deinit(alloc);

    var acc: usize = 0;
    const start = std.time.nanoTimestamp();
    for (0..num_queries) |i| {
        results.clearRetainingCapacity();
        try octree.queryRayIntersection(query_rays[i], &results);
        acc += results.items.len;
    }
    const end = std.time.nanoTimestamp();
    doNotOptimizeAway(acc);

    return @as(f64, @floatFromInt(end - start)) / @as(f64, @floatFromInt(num_queries));
}

pub fn main() !void {
    const args = try std.process.argsAlloc(std.heap.page_allocator);
    defer std.process.argsFree(std.heap.page_allocator, args);

    var num_primitives: usize = 10_000;
    var num_queries: usize = 100_000;
    var num_splits: usize = 1_000;
    var primitives_per_split: usize = 16;

    for (args) |arg| {
        if (std.mem.startsWith(u8, arg, "-N=")) {
            num_primitives = std.fmt.parseInt(usize, arg[3..], 10) catch 10_000;
        }
        if (std.mem.startsWith(u8, arg, "-Q=")) {
            num_queries = std.fmt.parseInt(usize, arg[3..], 10) catch 100_000;
        }
        if (std.mem.startsWith(u8, arg, "-S=")) {
            num_splits = std.fmt.parseInt(usize, arg[3..], 10) catch 1_000;
        }
        if (std.mem.startsWith(u8, arg, "-P=")) {
            primitives_per_split = std.fmt.parseInt(usize, arg[3..], 10) catch 16;
        }
    }

    const ts: i128 = std.time.nanoTimestamp();
    var prng = std.Random.DefaultPrng.init(@truncate(@as(u128, @bitCast(ts))));
    const rng = prng.random();

    const alloc = std.heap.page_allocator;

    print("\n Octree Benchmark (N={d}, Q={d}, S={d}, P={d}, ReleaseFast)\n", .{ num_primitives, num_queries, num_splits, primitives_per_split });
    print("{s}\n", .{"=" ** 60});
    print(" {s: <40} | {s: >15}\n", .{ "Operation", "Time/op" });
    print("{s}\n", .{"-" ** 60});

    var buf: [32]u8 = undefined;

    const insert_stats = try benchmarkInsert(num_primitives, rng, alloc);
    print(" {s: <40} | {s: >15}\n", .{ "Octree.insert", formatTime(insert_stats.time_ns, &buf) });

    const split_ns = try benchmarkSplitNode(num_splits, primitives_per_split, rng, alloc);
    print(" {s: <40} | {s: >15}\n", .{ "Octree.splitNode (direct)", formatTime(split_ns, &buf) });

    const query_aabb_ns = try benchmarkQueryAabb(num_primitives, num_queries, rng, alloc);
    print(" {s: <40} | {s: >15}\n", .{ "Octree.queryAabbOverlap", formatTime(query_aabb_ns, &buf) });

    const query_ray_ns = try benchmarkQueryRay(num_primitives, num_queries, rng, alloc);
    print(" {s: <40} | {s: >15}\n", .{ "Octree.queryRayIntersection", formatTime(query_ray_ns, &buf) });

    print("{s}\n", .{"=" ** 60});

    if (enable_diagnostics) {
        print("\n Tree Statistics\n", .{});
        print("{s}\n", .{"=" ** 60});
        print(" {s: <40} | {s: >15}\n", .{ "Metric", "Value" });
        print("{s}\n", .{"-" ** 60});
        print(" {s: <40} | {d: >15}\n", .{ "Total nodes", insert_stats.nodes.? });
        print(" {s: <40} | {d: >15}\n", .{ "Max depth", insert_stats.depth.? });
        print(" {s: <40} | {d: >15}\n", .{ "Primitive entries", insert_stats.entries.? });
        print(" {s: <40} | {d: >14.1}x\n", .{ "Duplication factor", insert_stats.duplication() });
        print("{s}\n", .{"=" ** 60});
    }
    print("\n", .{});
}
