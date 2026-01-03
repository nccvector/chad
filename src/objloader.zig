const std = @import("std");
const geometry = @import("geometry.zig");

const Mesh = geometry.Mesh;
const Model = geometry.Model;

/// OBJ file loader
pub const ObjLoader = struct {
    /// Load an OBJ file and return a Model with a single mesh
    /// Optional scale parameter (defaults to 1.0)
    pub fn loadModel(allocator: std.mem.Allocator, path: []const u8, scale: ?f32) !Model {
        const file = try std.fs.cwd().openFile(path, .{});
        defer file.close();

        const content = try file.readToEndAlloc(allocator, 64 * 1024 * 1024); // 64MB max
        defer allocator.free(content);

        return try parseObj(allocator, content, scale orelse 1.0);
    }

    /// Parse OBJ data from a string
    pub fn parseObj(allocator: std.mem.Allocator, content: []const u8, scale: f32) !Model {
        var vertices: std.ArrayListUnmanaged([3]f32) = .empty;
        defer vertices.deinit(allocator);
        var indices: std.ArrayListUnmanaged(u32) = .empty;
        defer indices.deinit(allocator);

        var lines = std.mem.splitScalar(u8, content, '\n');
        while (lines.next()) |raw_line| {
            const trimmed = std.mem.trim(u8, raw_line, " \t\r");
            if (trimmed.len == 0 or trimmed[0] == '#') continue;

            var iter = std.mem.splitScalar(u8, trimmed, ' ');
            const prefix = iter.next() orelse continue;

            if (std.mem.eql(u8, prefix, "v")) {
                // Vertex position (scaled)
                const x = (parseFloat(iter.next()) orelse continue) * scale;
                const y = (parseFloat(iter.next()) orelse continue) * scale;
                const z = (parseFloat(iter.next()) orelse continue) * scale;
                try vertices.append(allocator, .{ x, y, z });
            } else if (std.mem.eql(u8, prefix, "f")) {
                // Face - parse vertex indices
                var face_indices: [4]u32 = undefined;
                var face_count: usize = 0;

                while (iter.next()) |token| {
                    if (token.len == 0) continue;
                    var idx_iter = std.mem.splitScalar(u8, token, '/');
                    const idx_str = idx_iter.next() orelse continue;
                    const idx = std.fmt.parseInt(u32, idx_str, 10) catch continue;
                    if (face_count < 4) {
                        face_indices[face_count] = idx - 1; // OBJ is 1-based
                        face_count += 1;
                    }
                }

                // Triangulate (handles triangles and quads)
                if (face_count >= 3) {
                    try indices.append(allocator, face_indices[0]);
                    try indices.append(allocator, face_indices[1]);
                    try indices.append(allocator, face_indices[2]);
                }
                if (face_count == 4) {
                    try indices.append(allocator, face_indices[0]);
                    try indices.append(allocator, face_indices[2]);
                    try indices.append(allocator, face_indices[3]);
                }
            }
        }

        // Create mesh from parsed data
        const mesh = Mesh.init(
            allocator,
            try allocator.dupe([3]f32, vertices.items),
            try allocator.dupe(u32, indices.items),
        );

        // Create model and add mesh
        var model = Model.init(allocator);
        try model.addMesh(mesh);

        return model;
    }

    fn parseFloat(str: ?[]const u8) ?f32 {
        const s = str orelse return null;
        return std.fmt.parseFloat(f32, s) catch null;
    }
};
