const std = @import("std");
const za = @import("zalgebra");
const zopengl = @import("zopengl");

const chad = @import("geometry");
const geometry = chad.geometry;
const Mesh = geometry.Mesh;
const MeshId = geometry.MeshId;
const Model = geometry.Model;

// Use zalgebra types for 3D graphics math
const Vec3 = za.Vec3;
const Mat4 = za.Mat4;

const gl = zopengl.bindings;

// ============================================================================
// Mesh shader (position-based coloring)
// ============================================================================

const mesh_vertex_shader: [:0]const u8 =
    \\#version 330 core
    \\layout (location = 0) in vec3 aPos;
    \\uniform mat4 uMVP;
    \\out vec3 fragPos;
    \\void main() {
    \\    gl_Position = uMVP * vec4(aPos, 1.0);
    \\    fragPos = aPos;
    \\}
;

const mesh_fragment_shader: [:0]const u8 =
    \\#version 330 core
    \\in vec3 fragPos;
    \\out vec4 FragColor;
    \\void main() {
    \\    vec3 color = (fragPos + 1.0) * 0.5;
    \\    FragColor = vec4(color, 1.0);
    \\}
;

// ============================================================================
// Wireframe shader (uniform color)
// ============================================================================

const wireframe_vertex_shader: [:0]const u8 =
    \\#version 330 core
    \\layout (location = 0) in vec3 aPos;
    \\uniform mat4 uMVP;
    \\void main() {
    \\    gl_Position = uMVP * vec4(aPos, 1.0);
    \\}
;

const wireframe_fragment_shader: [:0]const u8 =
    \\#version 330 core
    \\uniform vec3 uColor;
    \\out vec4 FragColor;
    \\void main() {
    \\    FragColor = vec4(uColor, 1.0);
    \\}
;

/// GPU resources for a mesh
const GpuMesh = struct {
    vao: gl.Uint,
    vbo: gl.Uint,
    ebo: gl.Uint,
    index_count: gl.Sizei,

    fn deinit(self: *GpuMesh) void {
        gl.deleteVertexArrays(1, &self.vao);
        gl.deleteBuffers(1, &self.vbo);
        gl.deleteBuffers(1, &self.ebo);
    }
};

/// GPU resources for wireframe box (single instance, reused)
const WireframeBox = struct {
    vao: gl.Uint,
    vbo: gl.Uint,
    vertex_count: gl.Sizei,

    fn init() WireframeBox {
        // Unit cube wireframe: 12 edges, each edge is 2 vertices
        // Cube goes from -0.5 to 0.5 on each axis
        const h: f32 = 0.5;
        const vertices = [_][3]f32{
            // Bottom face edges
            .{ -h, -h, -h }, .{ h, -h, -h },
            .{ h, -h, -h },  .{ h, -h, h },
            .{ h, -h, h },   .{ -h, -h, h },
            .{ -h, -h, h },  .{ -h, -h, -h },
            // Top face edges
            .{ -h, h, -h },  .{ h, h, -h },
            .{ h, h, -h },   .{ h, h, h },
            .{ h, h, h },    .{ -h, h, h },
            .{ -h, h, h },   .{ -h, h, -h },
            // Vertical edges
            .{ -h, -h, -h }, .{ -h, h, -h },
            .{ h, -h, -h },  .{ h, h, -h },
            .{ h, -h, h },   .{ h, h, h },
            .{ -h, -h, h },  .{ -h, h, h },
        };

        var vao: gl.Uint = undefined;
        var vbo: gl.Uint = undefined;

        gl.genVertexArrays(1, &vao);
        gl.genBuffers(1, &vbo);

        gl.bindVertexArray(vao);

        gl.bindBuffer(gl.ARRAY_BUFFER, vbo);
        gl.bufferData(
            gl.ARRAY_BUFFER,
            @intCast(vertices.len * @sizeOf([3]f32)),
            &vertices,
            gl.STATIC_DRAW,
        );

        gl.vertexAttribPointer(0, 3, gl.FLOAT, gl.FALSE, @sizeOf([3]f32), @ptrFromInt(0));
        gl.enableVertexAttribArray(0);

        gl.bindVertexArray(0);

        return .{
            .vao = vao,
            .vbo = vbo,
            .vertex_count = vertices.len,
        };
    }

    fn deinit(self: *WireframeBox) void {
        gl.deleteVertexArrays(1, &self.vao);
        gl.deleteBuffers(1, &self.vbo);
    }
};

/// Camera with view and projection matrices
pub const Camera = struct {
    view: Mat4,
    projection: Mat4,

    pub fn init(fov_degrees: f32, aspect: f32, near: f32, far: f32) Camera {
        return .{
            .view = Mat4.identity(),
            .projection = Mat4.perspective(fov_degrees, aspect, near, far),
        };
    }

    pub fn setProjection(self: *Camera, fov_degrees: f32, aspect: f32, near: f32, far: f32) void {
        self.projection = Mat4.perspective(fov_degrees, aspect, near, far);
    }

    pub fn lookAt(self: *Camera, eye: Vec3, target: Vec3, up: Vec3) void {
        self.view = Mat4.lookAt(eye, target, up);
    }

    pub fn getViewProjection(self: *const Camera) Mat4 {
        return self.projection.mul(self.view);
    }
};

/// Debug visualizer for rendering geometry
pub const DebugVisualizer = struct {
    // Mesh rendering
    mesh_shader: gl.Uint,
    mesh_mvp_loc: gl.Int,
    gpu_meshes: std.AutoHashMap(MeshId, GpuMesh),

    // Wireframe rendering
    wireframe_shader: gl.Uint,
    wireframe_mvp_loc: gl.Int,
    wireframe_color_loc: gl.Int,
    wireframe_box: WireframeBox,

    camera: Camera,
    allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator, aspect_ratio: f32) !DebugVisualizer {
        // ====================================================================
        // Compile mesh shader
        // ====================================================================
        const mesh_shader = try compileShaderProgram(mesh_vertex_shader, mesh_fragment_shader);
        const mesh_mvp_loc = gl.getUniformLocation(mesh_shader, "uMVP");

        // ====================================================================
        // Compile wireframe shader
        // ====================================================================
        const wireframe_shader = try compileShaderProgram(wireframe_vertex_shader, wireframe_fragment_shader);
        const wireframe_mvp_loc = gl.getUniformLocation(wireframe_shader, "uMVP");
        const wireframe_color_loc = gl.getUniformLocation(wireframe_shader, "uColor");

        // ====================================================================
        // Create wireframe box geometry (kept in memory permanently)
        // ====================================================================
        const wireframe_box = WireframeBox.init();

        // Enable depth testing
        gl.enable(gl.DEPTH_TEST);

        // Initialize camera
        var camera = Camera.init(45.0, aspect_ratio, 0.1, 100.0);
        camera.lookAt(
            Vec3.new(0.0, 2.0, 5.0),
            Vec3.new(0.0, 0.0, 0.0),
            Vec3.new(0.0, 1.0, 0.0),
        );

        return .{
            .mesh_shader = mesh_shader,
            .mesh_mvp_loc = mesh_mvp_loc,
            .gpu_meshes = std.AutoHashMap(MeshId, GpuMesh).init(allocator),
            .wireframe_shader = wireframe_shader,
            .wireframe_mvp_loc = wireframe_mvp_loc,
            .wireframe_color_loc = wireframe_color_loc,
            .wireframe_box = wireframe_box,
            .camera = camera,
            .allocator = allocator,
        };
    }

    pub fn deinit(self: *DebugVisualizer) void {
        // Delete mesh GPU resources
        var iter = self.gpu_meshes.valueIterator();
        while (iter.next()) |gpu_mesh| {
            var m = gpu_mesh.*;
            m.deinit();
        }
        self.gpu_meshes.deinit();

        // Delete shaders
        gl.deleteProgram(self.mesh_shader);
        gl.deleteProgram(self.wireframe_shader);

        // Delete wireframe box
        self.wireframe_box.deinit();
    }

    /// Upload a mesh to the GPU (if not already uploaded)
    pub fn uploadMesh(self: *DebugVisualizer, mesh: *const Mesh) !void {
        if (self.gpu_meshes.contains(mesh.id)) {
            return; // Already uploaded
        }

        var vao: gl.Uint = undefined;
        var vbo: gl.Uint = undefined;
        var ebo: gl.Uint = undefined;

        gl.genVertexArrays(1, &vao);
        gl.genBuffers(1, &vbo);
        gl.genBuffers(1, &ebo);

        gl.bindVertexArray(vao);

        // Upload vertex data
        gl.bindBuffer(gl.ARRAY_BUFFER, vbo);
        gl.bufferData(
            gl.ARRAY_BUFFER,
            @intCast(mesh.vertices.len * @sizeOf([3]f32)),
            mesh.vertices.ptr,
            gl.STATIC_DRAW,
        );

        // Upload index data
        gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, ebo);
        gl.bufferData(
            gl.ELEMENT_ARRAY_BUFFER,
            @intCast(mesh.indices.len * @sizeOf(u32)),
            mesh.indices.ptr,
            gl.STATIC_DRAW,
        );

        // Position attribute (location 0)
        gl.vertexAttribPointer(0, 3, gl.FLOAT, gl.FALSE, @sizeOf([3]f32), @ptrFromInt(0));
        gl.enableVertexAttribArray(0);

        gl.bindVertexArray(0);

        try self.gpu_meshes.put(mesh.id, .{
            .vao = vao,
            .vbo = vbo,
            .ebo = ebo,
            .index_count = @intCast(mesh.indices.len),
        });
    }

    /// Upload all meshes in a model
    pub fn uploadModel(self: *DebugVisualizer, model: *const Model) !void {
        for (model.meshes.items) |*mesh| {
            try self.uploadMesh(mesh);
        }
    }

    /// Remove a mesh from the GPU
    pub fn removeMesh(self: *DebugVisualizer, mesh_id: MeshId) void {
        if (self.gpu_meshes.fetchRemove(mesh_id)) |entry| {
            var gpu_mesh = entry.value;
            gpu_mesh.deinit();
        }
    }

    /// Draw a model (converts lmao Mat4 to zalgebra Mat4)
    pub fn drawModel(self: *DebugVisualizer, model: *const Model) void {
        const vp = self.camera.getViewProjection();

        // Convert lmao Mat4 to zalgebra Mat4 (same memory layout)
        const model_transform = Mat4{ .data = model.transform.data };
        const mvp = vp.mul(model_transform);

        gl.useProgram(self.mesh_shader);
        gl.uniformMatrix4fv(self.mesh_mvp_loc, 1, gl.FALSE, @ptrCast(&mvp.data));

        for (model.meshes.items) |*mesh| {
            self.drawMesh(mesh.id);
        }
    }

    /// Draw a model with a custom transform (zalgebra Mat4)
    pub fn drawModelWithTransform(self: *DebugVisualizer, model: *const Model, transform: Mat4) void {
        const vp = self.camera.getViewProjection();
        const mvp = vp.mul(transform);

        gl.useProgram(self.mesh_shader);
        gl.uniformMatrix4fv(self.mesh_mvp_loc, 1, gl.FALSE, @ptrCast(&mvp.data));

        for (model.meshes.items) |*mesh| {
            self.drawMesh(mesh.id);
        }
    }

    /// Draw a single mesh by ID
    pub fn drawMesh(self: *DebugVisualizer, mesh_id: MeshId) void {
        if (self.gpu_meshes.get(mesh_id)) |gpu_mesh| {
            gl.bindVertexArray(gpu_mesh.vao);
            gl.drawElements(gl.TRIANGLES, gpu_mesh.index_count, gl.UNSIGNED_INT, null);
            gl.bindVertexArray(0);
        }
    }

    /// Draw a wireframe box at the specified position, size, and orientation
    /// - position: center of the box in world space
    /// - size: dimensions (width, height, depth)
    /// - orientation: rotation angles in degrees (yaw, pitch, roll) = (Y, X, Z)
    /// - color: RGB color (0.0 to 1.0)
    pub fn drawBoxWireframe(
        self: *DebugVisualizer,
        position: Vec3,
        size: Vec3,
        orientation: Vec3,
        color: Vec3,
    ) void {
        // Build model matrix: Translation * Rotation * Scale
        // Rotation order: Y (yaw) -> X (pitch) -> Z (roll)
        const scale_mat = Mat4.fromScale(size);
        const rot_y = Mat4.fromRotation(orientation.x(), Vec3.new(0, 1, 0)); // yaw
        const rot_x = Mat4.fromRotation(orientation.y(), Vec3.new(1, 0, 0)); // pitch
        const rot_z = Mat4.fromRotation(orientation.z(), Vec3.new(0, 0, 1)); // roll
        const rotation_mat = rot_y.mul(rot_x).mul(rot_z);
        const translation_mat = Mat4.fromTranslate(position);

        const model_mat = translation_mat.mul(rotation_mat).mul(scale_mat);
        const mvp = self.camera.getViewProjection().mul(model_mat);

        // Use wireframe shader
        gl.useProgram(self.wireframe_shader);
        gl.uniformMatrix4fv(self.wireframe_mvp_loc, 1, gl.FALSE, @ptrCast(&mvp.data));
        gl.uniform3f(self.wireframe_color_loc, color.x(), color.y(), color.z());

        // Draw the wireframe box
        gl.bindVertexArray(self.wireframe_box.vao);
        gl.drawArrays(gl.LINES, 0, self.wireframe_box.vertex_count);
        gl.bindVertexArray(0);
    }

    /// Draw an axis-aligned bounding box as wireframe
    /// - min: minimum corner of the AABB
    /// - max: maximum corner of the AABB
    /// - color: RGB color (0.0 to 1.0)
    pub fn drawAabbWireframe(
        self: *DebugVisualizer,
        min: Vec3,
        max: Vec3,
        color: Vec3,
    ) void {
        const center = min.add(max).scale(0.5);
        const size = max.sub(min);
        self.drawBoxWireframe(center, size, Vec3.new(0, 0, 0), color);
    }

    /// Begin a frame (clear buffers)
    pub fn beginFrame(self: *DebugVisualizer, width: u32, height: u32) void {
        _ = self;
        gl.viewport(0, 0, @intCast(width), @intCast(height));
        gl.clearColor(0.1, 0.1, 0.1, 1.0);
        gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    }

    fn compileShaderProgram(vertex_src: [:0]const u8, fragment_src: [:0]const u8) !gl.Uint {
        // Compile vertex shader
        const vertex_shader = gl.createShader(gl.VERTEX_SHADER);
        defer gl.deleteShader(vertex_shader);
        const vs_sources = [_][*c]const u8{vertex_src.ptr};
        gl.shaderSource(vertex_shader, 1, &vs_sources, null);
        gl.compileShader(vertex_shader);

        if (!checkShaderCompilation(vertex_shader, "vertex")) {
            return error.VertexShaderCompilationFailed;
        }

        // Compile fragment shader
        const fragment_shader = gl.createShader(gl.FRAGMENT_SHADER);
        defer gl.deleteShader(fragment_shader);
        const fs_sources = [_][*c]const u8{fragment_src.ptr};
        gl.shaderSource(fragment_shader, 1, &fs_sources, null);
        gl.compileShader(fragment_shader);

        if (!checkShaderCompilation(fragment_shader, "fragment")) {
            return error.FragmentShaderCompilationFailed;
        }

        // Link program
        const program = gl.createProgram();
        gl.attachShader(program, vertex_shader);
        gl.attachShader(program, fragment_shader);
        gl.linkProgram(program);

        if (!checkProgramLinking(program)) {
            gl.deleteProgram(program);
            return error.ShaderProgramLinkingFailed;
        }

        return program;
    }

    fn checkShaderCompilation(shader: gl.Uint, shader_type: []const u8) bool {
        var success: gl.Int = undefined;
        gl.getShaderiv(shader, gl.COMPILE_STATUS, &success);
        if (success == 0) {
            var info_log: [512]u8 = undefined;
            gl.getShaderInfoLog(shader, 512, null, &info_log);
            std.log.err("{s} shader compilation failed: {s}", .{ shader_type, &info_log });
            return false;
        }
        return true;
    }

    fn checkProgramLinking(program: gl.Uint) bool {
        var success: gl.Int = undefined;
        gl.getProgramiv(program, gl.LINK_STATUS, &success);
        if (success == 0) {
            var info_log: [512]u8 = undefined;
            gl.getProgramInfoLog(program, 512, null, &info_log);
            std.log.err("Shader program linking failed: {s}", .{&info_log});
            return false;
        }
        return true;
    }
};
