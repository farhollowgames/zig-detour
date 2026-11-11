//! Differential tests of the search nodes: the node's layout and the packing
//! of its parent index, state and flags, the node pool and the node queue.
//! Both sides get the same calls, and every node handed back, every field,
//! every bucket chain and every pop must then be the same.

const std = @import("std");
const builtin = @import("builtin");
const detour = @import("detour");
const support = @import("support.zig");
const reference = support.reference;

const testing = std.testing;

const Node = detour.Node;
const NodePool = detour.NodePool;
const NodeQueue = detour.NodeQueue;
const NodeFlags = detour.node.NodeFlags;
const PolyRef = detour.PolyRef;

const native_endian = builtin.cpu.arch.endian();

/// The room of a buffer one node of either side is laid over. More than a
/// node takes, so a write past the node's end shows in the bytes behind it.
const node_bytes_len = 64;

/// The bits of the packed word that belong to no field. The original may
/// leave anything there, so byte comparisons clear them on both sides first.
const unused_bits: u32 = @bitCast(Node.Bits{ .reserved = std.math.maxInt(u3) });

/// Every field of one node as plain values, read from either side.
const NodeFields = struct {
    pos: [3]f32,
    cost: f32,
    total: f32,
    parent_index: u32,
    state: u32,
    flags: u32,
    id: PolyRef,

    fn ofReference(node: *const reference.Node) NodeFields {
        var fields: NodeFields = undefined;
        reference.ref_node_fields(
            node,
            &fields.pos,
            &fields.cost,
            &fields.total,
            &fields.parent_index,
            &fields.state,
            &fields.flags,
            &fields.id,
        );
        return fields;
    }

    fn ofPort(node: *const Node) NodeFields {
        return .{
            .pos = node.pos,
            .cost = node.cost,
            .total = node.total,
            .parent_index = node.bits.parent_index,
            .state = node.bits.state,
            .flags = @as(u3, @bitCast(node.bits.flags)),
            .id = node.id,
        };
    }

    /// Writes every field through the original's own assignments.
    fn writeReference(fields: NodeFields, node: *reference.Node) void {
        reference.ref_node_set(
            node,
            &fields.pos,
            fields.cost,
            fields.total,
            fields.parent_index,
            fields.state,
            fields.flags,
            fields.id,
        );
    }

    /// Writes every field one by one, the way the port's searches do.
    fn writePort(fields: NodeFields, node: *Node) void {
        node.pos = fields.pos;
        node.cost = fields.cost;
        node.total = fields.total;
        node.bits.parent_index = @intCast(fields.parent_index);
        node.bits.state = @intCast(fields.state);
        node.bits.flags = @bitCast(@as(u3, @intCast(fields.flags)));
        node.id = fields.id;
    }

    /// Fails unless both hold the same bits in every field. Floats compare
    /// by their bits, so a value that is not a number equals itself.
    fn expectSame(original: NodeFields, port: NodeFields) !void {
        errdefer std.debug.print("  original {any}\n  port     {any}\n", .{ original, port });
        for (original.pos, port.pos) |a, b| try expectSameBits(a, b);
        try expectSameBits(original.cost, port.cost);
        try expectSameBits(original.total, port.total);
        try testing.expectEqual(original.parent_index, port.parent_index);
        try testing.expectEqual(original.state, port.state);
        try testing.expectEqual(original.flags, port.flags);
        try testing.expectEqual(original.id, port.id);
    }

    /// Fields of any value each one can hold.
    fn random(source: std.Random) NodeFields {
        return .{
            .pos = .{ randomFloat(source), randomFloat(source), randomFloat(source) },
            .cost = randomFloat(source),
            .total = randomFloat(source),
            .parent_index = source.int(u24),
            .state = source.int(u2),
            .flags = source.int(u3),
            .id = source.int(PolyRef),
        };
    }

    /// Fields that differ from slot to slot, so a node written to the wrong
    /// slot shows.
    fn ofSlot(slot: u32) NodeFields {
        const number: f32 = @floatFromInt(slot);
        return .{
            .pos = .{ number + 0.25, -number, number * 0.5 - 3.0 },
            .cost = number * 1.5 + 7.0,
            .total = -number - 0.125,
            .parent_index = (slot *% 7919 +% 13) & 0xffffff,
            .state = slot & 3,
            .flags = (slot >> 2) & 7,
            .id = @as(PolyRef, slot) *% 2654435761 +% 0x5a5a,
        };
    }
};

/// A buffer one node of either side is laid over, to compare the bytes each
/// side's writes leave.
const NodeBytes = struct {
    bytes: [node_bytes_len]u8 align(8),

    fn filled(byte: u8) NodeBytes {
        return .{ .bytes = @splat(byte) };
    }

    fn port(self: *NodeBytes) *Node {
        return @ptrCast(&self.bytes);
    }

    fn original(self: *NodeBytes) *reference.Node {
        return @ptrCast(&self.bytes);
    }

    /// The bytes with the bits no field owns cleared.
    fn comparable(self: *const NodeBytes) [node_bytes_len]u8 {
        var bytes = self.bytes;
        const word = bytes[@offsetOf(Node, "bits")..][0..4];
        const value = std.mem.readInt(u32, word, native_endian) & ~unused_bits;
        std.mem.writeInt(u32, word, value, native_endian);
        return bytes;
    }

    /// Fails unless both buffers hold the same bytes, apart from the bits
    /// no field owns.
    fn expectSame(from_original: *const NodeBytes, from_port: *const NodeBytes) !void {
        const expected = from_original.comparable();
        const actual = from_port.comparable();
        try testing.expectEqualSlices(u8, &expected, &actual);
    }
};

/// Fails unless the two floats hold the same bits.
fn expectSameBits(original: f32, port: f32) !void {
    try testing.expectEqual(@as(u32, @bitCast(original)), @as(u32, @bitCast(port)));
}

/// A float of any kind a node may be given: zeroes of both signs, both
/// infinities, values that are not numbers, any bit pattern and plain costs.
fn randomFloat(random: std.Random) f32 {
    return switch (random.uintLessThan(u32, 8)) {
        0 => 0.0,
        1 => -0.0,
        2 => std.math.inf(f32),
        3 => -std.math.inf(f32),
        4 => std.math.nan(f32),
        5 => @as(f32, @bitCast(random.int(u32))),
        else => (random.float(f32) - 0.5) * 2000.0,
    };
}

// ---------------------------------------------------------------------------
// Layout
// ---------------------------------------------------------------------------

test "Node: the size and the field offsets are the original's" {
    try testing.expectEqual(reference.ref_sizeof_node(), @as(usize, @sizeOf(Node)));
    try testing.expectEqual(reference.ref_offsetof_node_pos(), @as(usize, @offsetOf(Node, "pos")));
    try testing.expectEqual(
        reference.ref_offsetof_node_cost(),
        @as(usize, @offsetOf(Node, "cost")),
    );
    try testing.expectEqual(
        reference.ref_offsetof_node_total(),
        @as(usize, @offsetOf(Node, "total")),
    );
    try testing.expectEqual(reference.ref_offsetof_node_id(), @as(usize, @offsetOf(Node, "id")));

    // The original gives its packed word no name to take an offset of: it is
    // the 32 bits between the total and the id.
    try testing.expectEqual(
        reference.ref_offsetof_node_total() + 4,
        @as(usize, @offsetOf(Node, "bits")),
    );
    try testing.expectEqual(@as(usize, 4), @as(usize, @sizeOf(Node.Bits)));
    try testing.expect(@offsetOf(Node, "bits") + 4 <= reference.ref_offsetof_node_id());
    try testing.expect(reference.ref_sizeof_node() <= node_bytes_len);
}

test "node constants: the flag bits, the null index and the bit widths are the original's" {
    const open: u3 = @bitCast(NodeFlags{ .open = true });
    const closed: u3 = @bitCast(NodeFlags{ .closed = true });
    const parent_detached: u3 = @bitCast(NodeFlags{ .parent_detached = true });
    try testing.expectEqual(reference.ref_const_node_open(), @as(u32, open));
    try testing.expectEqual(reference.ref_const_node_closed(), @as(u32, closed));
    try testing.expectEqual(reference.ref_const_node_parent_detached(), @as(u32, parent_detached));

    try testing.expectEqual(reference.ref_const_null_idx(), @as(u32, detour.node.null_index));
    try testing.expectEqual(
        reference.ref_sizeof_node_index(),
        @as(usize, @sizeOf(detour.node.NodeIndex)),
    );
    try testing.expectEqual(
        reference.ref_const_node_parent_bits(),
        @as(i32, detour.node.node_parent_bits),
    );
    try testing.expectEqual(
        reference.ref_const_node_state_bits(),
        @as(i32, detour.node.node_state_bits),
    );
    try testing.expectEqual(
        reference.ref_const_max_states_per_node(),
        @as(i32, detour.node.node_states_max),
    );

    // The fields are as wide as the constants say and in the original's
    // order: the parent index lowest, then the state, then the flags.
    try testing.expectEqual(@as(i32, 0), @as(i32, @bitOffsetOf(Node.Bits, "parent_index")));
    try testing.expectEqual(
        reference.ref_const_node_parent_bits(),
        @as(i32, @bitOffsetOf(Node.Bits, "state")),
    );
    try testing.expectEqual(
        reference.ref_const_node_parent_bits() + reference.ref_const_node_state_bits(),
        @as(i32, @bitOffsetOf(Node.Bits, "flags")),
    );
    try testing.expectEqual(@as(i32, 3), @as(i32, @bitSizeOf(NodeFlags)));
    try testing.expectEqual(@as(i32, 29), @as(i32, @bitOffsetOf(Node.Bits, "reserved")));
}

// ---------------------------------------------------------------------------
// Bit packing
// ---------------------------------------------------------------------------

/// Parent indices that reach every byte of the 24 bits: none, the first
/// node, single bits, mixed patterns and the largest.
const parent_indices = [_]u32{
    0,       1,        2,        0xff,     0x100,    0xffff,
    0x10000, 0x5a5a5a, 0xa5a5a5, 0x800000, 0xfffffe, 0xffffff,
};

/// Writes the three packed fields one at a time on each side, over buffers
/// that start out as `fill`, and compares the bytes after every write. Each
/// side then reads the fields out of the other's bytes.
fn expectSamePacking(fill: u8, parent_index: u32, state: u32, flags: u32) !void {
    var from_original: NodeBytes = .filled(fill);
    var from_port: NodeBytes = .filled(fill);

    reference.ref_node_set_pidx(from_original.original(), parent_index);
    from_port.port().bits.parent_index = @intCast(parent_index);
    try from_original.expectSame(&from_port);

    reference.ref_node_set_state(from_original.original(), state);
    from_port.port().bits.state = @intCast(state);
    try from_original.expectSame(&from_port);

    reference.ref_node_set_flags(from_original.original(), flags);
    from_port.port().bits.flags = @bitCast(@as(u3, @intCast(flags)));
    try from_original.expectSame(&from_port);

    // The original's getters over the port's bytes.
    try testing.expectEqual(parent_index, reference.ref_node_get_pidx(from_port.original()));
    try testing.expectEqual(state, reference.ref_node_get_state(from_port.original()));
    try testing.expectEqual(flags, reference.ref_node_get_flags(from_port.original()));

    // The port's fields over the original's bytes.
    const bits = from_original.port().bits;
    try testing.expectEqual(parent_index, @as(u32, bits.parent_index));
    try testing.expectEqual(state, @as(u32, bits.state));
    try testing.expectEqual(flags, @as(u32, @as(u3, @bitCast(bits.flags))));

    // The original's bytes as one word: each field sits where the port's
    // packed word puts it.
    const expected_word: u32 = @bitCast(Node.Bits{
        .parent_index = @intCast(parent_index),
        .state = @intCast(state),
        .flags = @bitCast(@as(u3, @intCast(flags))),
    });
    const word = from_original.bytes[@offsetOf(Node, "bits")..][0..4];
    try testing.expectEqual(
        expected_word,
        std.mem.readInt(u32, word, native_endian) & ~unused_bits,
    );

    // Neither side wrote anything but the packed word.
    for ([_]*const NodeBytes{ &from_original, &from_port }) |buffer| {
        for (buffer.bytes, 0..) |byte, i| {
            const in_word = i >= @offsetOf(Node, "bits") and i < @offsetOf(Node, "bits") + 4;
            if (!in_word) try testing.expectEqual(fill, byte);
        }
    }
}

test "Node.Bits: parent index, state and flags pack into the original's bytes" {
    try testing.expect(reference.ref_sizeof_node() <= node_bytes_len);

    // A buffer of zeroes shows the bits a write sets, one of ones the bits
    // it clears, and a mixed one both.
    for ([_]u8{ 0x00, 0xff, 0xa5 }) |fill| {
        for (parent_indices) |parent_index| {
            for (0..4) |state| {
                for (0..8) |flags| {
                    errdefer std.debug.print(
                        "  fill 0x{x}, parent index 0x{x}, state {d}, flags {d}\n",
                        .{ fill, parent_index, state, flags },
                    );
                    try expectSamePacking(fill, parent_index, @intCast(state), @intCast(flags));
                }
            }
        }
    }
}

test "Node.Bits: the original's getters read every field the port sets alone" {
    // One field at its largest and the others zero, and the other way
    // round: a field that leaks into its neighbour shows in the getters.
    const cases = [_][3]u32{
        .{ 0xffffff, 0, 0 }, .{ 0, 3, 0 },        .{ 0, 0, 7 },
        .{ 0, 3, 7 },        .{ 0xffffff, 0, 7 }, .{ 0xffffff, 3, 0 },
        .{ 0xffffff, 3, 7 }, .{ 0, 0, 0 },
    };
    for (cases) |case| {
        var from_port: NodeBytes = .filled(0);
        from_port.port().bits = .{
            .parent_index = @intCast(case[0]),
            .state = @intCast(case[1]),
            .flags = @bitCast(@as(u3, @intCast(case[2]))),
        };
        try testing.expectEqual(case[0], reference.ref_node_get_pidx(from_port.original()));
        try testing.expectEqual(case[1], reference.ref_node_get_state(from_port.original()));
        try testing.expectEqual(case[2], reference.ref_node_get_flags(from_port.original()));

        var from_original: NodeBytes = .filled(0);
        reference.ref_node_set_pidx(from_original.original(), case[0]);
        reference.ref_node_set_state(from_original.original(), case[1]);
        reference.ref_node_set_flags(from_original.original(), case[2]);
        try from_original.expectSame(&from_port);
    }

    // Each named flag is the bit the original gives it.
    const named = [_]struct { flags: NodeFlags, bit: u32 }{
        .{ .flags = .{ .open = true }, .bit = reference.ref_const_node_open() },
        .{ .flags = .{ .closed = true }, .bit = reference.ref_const_node_closed() },
        .{
            .flags = .{ .parent_detached = true },
            .bit = reference.ref_const_node_parent_detached(),
        },
    };
    for (named) |flag| {
        var from_port: NodeBytes = .filled(0);
        from_port.port().bits.flags = flag.flags;
        try testing.expectEqual(flag.bit, reference.ref_node_get_flags(from_port.original()));
    }
}

test "Node: whole random nodes written by each side leave the same bytes" {
    try testing.expect(reference.ref_sizeof_node() <= node_bytes_len);

    const seed: u64 = 0x6e6f6465;
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(seed));
    const random = prng.random();

    for (0..4000) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const fields: NodeFields = .random(random);
        const fill = random.int(u8);
        var from_original: NodeBytes = .filled(fill);
        var from_port: NodeBytes = .filled(fill);

        fields.writeReference(from_original.original());
        fields.writePort(from_port.port());
        try from_original.expectSame(&from_port);

        // Each side reads back what the other wrote.
        try fields.expectSame(NodeFields.ofReference(from_port.original()));
        try fields.expectSame(NodeFields.ofPort(from_original.port()));

        // Neither side wrote past its node.
        for (from_original.bytes[@sizeOf(Node)..]) |byte| try testing.expectEqual(fill, byte);
        for (from_port.bytes[@sizeOf(Node)..]) |byte| try testing.expectEqual(fill, byte);
    }
}

// ---------------------------------------------------------------------------
// Node pool
// ---------------------------------------------------------------------------

/// The most nodes a `findNodes` call of these tests has room for.
const find_room_max = 8;

/// What every entry of a `findNodes` buffer points at before the call, on
/// both sides, to show which entries the call left alone.
var untouched_node: Node = std.mem.zeroes(Node);

/// One node pool on each side, set up alike and given the same calls.
const PoolPair = struct {
    pool: NodePool,
    reference_pool: *reference.NodePool,
    /// The original's first node. Its nodes are told apart by how far they
    /// lie behind this one.
    reference_first: *reference.Node,

    /// Readies both pools and gives every slot the same fields on both
    /// sides. The original hands its nodes out as the allocator left them
    /// and the port zeroes them, and neither writes a position when it
    /// hands a node out, so without this the fields could not be compared.
    fn init(gpa: std.mem.Allocator, node_count_max: i32, hash_size: i32) !PoolPair {
        const reference_pool = reference.ref_node_pool_create(node_count_max, hash_size) orelse
            return error.OutOfMemory;
        errdefer reference.ref_node_pool_destroy(reference_pool);
        const reference_first = reference.ref_node_pool_get_node_at_idx(reference_pool, 1) orelse
            return error.TestUnexpectedResult;

        var pair: PoolPair = .{
            .pool = .empty,
            .reference_pool = reference_pool,
            .reference_first = reference_first,
        };
        try pair.pool.init(gpa, node_count_max, hash_size);

        for (pair.pool.nodes, 0..) |*node, slot| {
            const fields: NodeFields = .ofSlot(@intCast(slot));
            fields.writeReference(pair.referenceNode(@intCast(slot)));
            fields.writePort(node);
        }
        return pair;
    }

    fn deinit(pair: *PoolPair, gpa: std.mem.Allocator) void {
        pair.pool.deinit(gpa);
        reference.ref_node_pool_destroy(pair.reference_pool);
    }

    /// The original's node in `slot`, handed out or not.
    fn referenceNode(pair: *const PoolPair, slot: u32) *reference.Node {
        const address = @intFromPtr(pair.reference_first) + slot * reference.ref_sizeof_node();
        return @ptrFromInt(address);
    }

    /// The slot of one of the original's nodes, -1 for no node.
    fn referenceSlot(pair: *const PoolPair, node: ?*const reference.Node) i64 {
        const found = node orelse return -1;
        const offset = @intFromPtr(found) -% @intFromPtr(pair.reference_first);
        return @intCast(offset / reference.ref_sizeof_node());
    }

    /// The slot of one of the port's nodes, -1 for no node.
    fn portSlot(pair: *const PoolPair, node: ?*const Node) i64 {
        const found = node orelse return -1;
        const offset = @intFromPtr(found) -% @intFromPtr(pair.pool.nodes.ptr);
        return @intCast(offset / @sizeOf(Node));
    }

    /// Fails unless both sides handed back the same slot, or both no node,
    /// with the same fields in it, the same number for it from `getNodeIdx`
    /// and the same node for that number from `getNodeAtIdx`. Returns the
    /// slot, null for no node.
    fn expectSameNode(pair: *const PoolPair, original: ?*reference.Node, port: ?*Node) !?u32 {
        const original_slot = pair.referenceSlot(original);
        const port_slot = pair.portSlot(port);
        if (original_slot != port_slot) {
            std.debug.print(
                "  node: original slot {d}, port slot {d}\n",
                .{ original_slot, port_slot },
            );
            return error.TestExpectedEqual;
        }

        // Both sides number a node one above its slot, and no node zero.
        const original_idx = reference.ref_node_pool_get_node_idx(pair.reference_pool, original);
        const port_idx = pair.pool.getNodeIdx(port);
        if (original_idx != port_idx or original_idx != port_slot + 1) {
            std.debug.print(
                "  getNodeIdx(slot {d}): original {d}, port {d}\n",
                .{ port_slot, original_idx, port_idx },
            );
            return error.TestExpectedEqual;
        }

        // And the number leads back to the node.
        const original_back = reference.ref_node_pool_get_node_at_idx(
            pair.reference_pool,
            original_idx,
        );
        try testing.expectEqual(@intFromPtr(original), @intFromPtr(original_back));
        try testing.expectEqual(@intFromPtr(port), @intFromPtr(pair.pool.getNodeAtIdx(port_idx)));

        const original_node = original orelse return null;
        const port_node = port orelse return null;
        try NodeFields.ofReference(original_node).expectSame(NodeFields.ofPort(port_node));
        return @intCast(port_slot);
    }

    /// Takes the node of `id` in `state` on both sides; the slot both gave,
    /// null when both were full.
    fn getNode(pair: *PoolPair, id: PolyRef, state: u8) !?u32 {
        errdefer std.debug.print("  getNode({d}, {d})\n", .{ id, state });
        const original = reference.ref_node_pool_get_node(pair.reference_pool, id, state);
        const port = pair.pool.getNode(id, state);
        const slot = try pair.expectSameNode(original, port);
        try pair.expectSameCount();
        return slot;
    }

    /// Looks the node of `id` in `state` up on both sides; the slot both
    /// found, null when neither has one.
    fn findNode(pair: *const PoolPair, id: PolyRef, state: u8) !?u32 {
        errdefer std.debug.print("  findNode({d}, {d})\n", .{ id, state });
        const original = reference.ref_node_pool_find_node(pair.reference_pool, id, state);
        const port = pair.pool.findNode(id, state);
        const slot = try pair.expectSameNode(original, port);
        try pair.expectSameCount();
        return slot;
    }

    /// Asks both sides for every node of `id` with room for `room` of them,
    /// and fails unless they give the same count and the same slots in the
    /// same order and leave the rest of their buffers alone. Returns the
    /// count.
    fn findNodes(pair: *const PoolPair, id: PolyRef, room: usize) !usize {
        errdefer std.debug.print("  findNodes({d}) with room for {d}\n", .{ id, room });
        const untouched_original: *reference.Node = @ptrCast(&untouched_node);
        var original_nodes: [find_room_max]?*reference.Node = @splat(untouched_original);
        var port_nodes: [find_room_max]*Node = @splat(&untouched_node);

        const original_count: usize = reference.ref_node_pool_find_nodes(
            pair.reference_pool,
            id,
            &original_nodes,
            @intCast(room),
        );
        const port_count = pair.pool.findNodes(id, port_nodes[0..room]);
        if (original_count != port_count) {
            std.debug.print(
                "  findNodes: original {d}, port {d}\n",
                .{ original_count, port_count },
            );
            return error.TestExpectedEqual;
        }
        try testing.expect(port_count <= room);

        for (original_nodes, port_nodes, 0..) |original, port, i| {
            const original_untouched = @intFromPtr(original) == @intFromPtr(untouched_original);
            const port_untouched = port == &untouched_node;
            if (i < port_count) {
                try testing.expect(!original_untouched);
                try testing.expect(!port_untouched);
                _ = try pair.expectSameNode(original, port);
                try testing.expectEqual(id, port.id);
            } else {
                try testing.expect(original_untouched);
                try testing.expect(port_untouched);
            }
        }
        try pair.expectSameCount();
        return port_count;
    }

    fn clear(pair: *PoolPair) void {
        reference.ref_node_pool_clear(pair.reference_pool);
        pair.pool.clear();
    }

    /// Writes `fields` into `slot` on both sides, the way a search fills in
    /// a node it was handed.
    fn writeNode(pair: *PoolPair, slot: u32, fields: NodeFields) void {
        fields.writeReference(pair.referenceNode(slot));
        fields.writePort(&pair.pool.nodes[slot]);
    }

    /// Fails unless both pools have handed out the same number of nodes.
    fn expectSameCount(pair: *const PoolPair) !void {
        const original = reference.ref_node_pool_get_node_count(pair.reference_pool);
        const port = pair.pool.getNodeCount();
        if (original != port) {
            std.debug.print("  getNodeCount(): original {d}, port {d}\n", .{ original, port });
            return error.TestExpectedEqual;
        }
    }

    /// Fails unless the two pools have the same sizes and counts, the same
    /// fields in every slot, handed out or not, and the same chain of nodes
    /// in every bucket.
    fn expectSamePool(pair: *const PoolPair) !void {
        const pool = &pair.pool;
        const original = pair.reference_pool;

        try pair.expectSameCount();
        try testing.expectEqual(
            reference.ref_node_pool_get_max_nodes(original),
            pool.getMaxNodes(),
        );
        try testing.expectEqual(
            reference.ref_node_pool_get_hash_size(original),
            pool.getHashSize(),
        );
        try testing.expectEqual(reference.ref_node_pool_get_mem_used(original), pool.getMemUsed());

        for (pool.nodes, 0..) |*node, slot| {
            errdefer std.debug.print("  slot {d}\n", .{slot});
            const original_node = pair.referenceNode(@intCast(slot));
            try NodeFields.ofReference(original_node).expectSame(NodeFields.ofPort(node));
            try testing.expectEqual(
                reference.ref_node_pool_get_next(original, @intCast(slot)),
                pool.getNext(@intCast(slot)),
            );
        }

        const node_count = pool.getNodeCount();
        var chained: i32 = 0;
        var bucket: i32 = 0;
        while (bucket < pool.getHashSize()) : (bucket += 1) {
            errdefer std.debug.print("  bucket {d}\n", .{bucket});
            var original_i = reference.ref_node_pool_get_first(original, bucket);
            var port_i = pool.getFirst(bucket);
            var steps: i32 = 0;
            while (true) : (steps += 1) {
                try testing.expectEqual(original_i, port_i);
                if (port_i == detour.node.null_index) break;
                // A chain names only nodes that are handed out, and one
                // longer than the pool has run into a loop.
                try testing.expect(port_i < node_count);
                try testing.expect(steps < node_count);
                chained += 1;
                original_i = reference.ref_node_pool_get_next(original, original_i);
                port_i = pool.getNext(port_i);
            }
        }

        // Every node handed out is in one chain.
        try testing.expectEqual(node_count, chained);
    }
};

/// One pool size the sized tests run over.
const PoolSize = struct {
    node_count_max: i32,
    hash_size: i32,

    /// How many nodes the sized tests take from the pool: all of them,
    /// unless the pool chains every node into one bucket, where each call
    /// walks the whole chain and a large pool would take minutes to fill.
    fn takeCount(size: PoolSize) u32 {
        const node_count_max: u32 = @intCast(size.node_count_max);
        return if (size.hash_size == 1) @min(node_count_max, 400) else node_count_max;
    }
};

/// The smallest pools with every hash size they are given in practice,
/// pools with fewer buckets than nodes and with more, and the largest pool
/// there is.
const pool_sizes = [_]PoolSize{
    .{ .node_count_max = 1, .hash_size = 1 },
    .{ .node_count_max = 1, .hash_size = 2 },
    .{ .node_count_max = 2, .hash_size = 1 },
    .{ .node_count_max = 2, .hash_size = 2 },
    .{ .node_count_max = 3, .hash_size = 1 },
    .{ .node_count_max = 3, .hash_size = 4 },
    .{ .node_count_max = 4, .hash_size = 1 },
    .{ .node_count_max = 4, .hash_size = 2 },
    .{ .node_count_max = 4, .hash_size = 8 },
    .{ .node_count_max = 64, .hash_size = 32 },
    .{ .node_count_max = 100, .hash_size = 1 },
    .{ .node_count_max = 1000, .hash_size = 256 },
    .{ .node_count_max = 4096, .hash_size = 1024 },
    .{ .node_count_max = 65535, .hash_size = 1 },
    .{ .node_count_max = 65535, .hash_size = 65536 },
    .{ .node_count_max = 65535, .hash_size = 1 << 17 },
};

/// An odd multiplier, so `spreadId` gives every number its own id.
const id_multiplier: PolyRef = if (@bitSizeOf(PolyRef) == 64) 0x9e3779b97f4a7c15 else 0x9e3779b1;

/// A polygon id for `number` with bits all over the id, so the ids of a
/// test land in every bucket. Different numbers give different ids.
fn spreadId(number: u32) PolyRef {
    return @as(PolyRef, number) *% id_multiplier +% 1;
}

/// Fails unless a pool call gave the node in slot `expected`.
fn expectSlot(expected: u32, actual: ?u32) !void {
    const slot = actual orelse {
        std.debug.print("  no node, expected slot {d}\n", .{expected});
        return error.TestExpectedEqual;
    };
    try testing.expectEqual(expected, slot);
}

/// Fails unless a pool call gave no node.
fn expectNoSlot(actual: ?u32) !void {
    if (actual) |slot| {
        std.debug.print("  slot {d}, expected no node\n", .{slot});
        return error.TestExpectedEqual;
    }
}

test "NodePool.init: a new pool has the original's sizes, empty buckets and no nodes" {
    const gpa = testing.allocator;
    for (pool_sizes) |size| {
        errdefer std.debug.print(
            "  {d} nodes, {d} buckets\n",
            .{ size.node_count_max, size.hash_size },
        );
        var pair = try PoolPair.init(gpa, size.node_count_max, size.hash_size);
        defer pair.deinit(gpa);

        try pair.expectSamePool();
        try testing.expectEqual(@as(i32, 0), pair.pool.getNodeCount());
        try testing.expectEqual(size.node_count_max, pair.pool.getMaxNodes());
        try testing.expectEqual(size.hash_size, pair.pool.getHashSize());

        // What is left of the size with the three arrays taken off is the
        // original's pool object itself.
        const node_count_max: i64 = size.node_count_max;
        const hash_size: i64 = size.hash_size;
        const arrays = (@sizeOf(Node) + 2) * node_count_max + 2 * hash_size;
        try testing.expectEqual(
            @as(i64, @intCast(reference.ref_sizeof_node_pool())),
            pair.pool.getMemUsed() - arrays,
        );

        // An empty pool finds nothing, whatever is asked.
        for ([_]PolyRef{ 0, 1, spreadId(5), std.math.maxInt(PolyRef) }) |id| {
            for (0..4) |state| try expectNoSlot(try pair.findNode(id, @intCast(state)));
            try testing.expectEqual(@as(usize, 0), try pair.findNodes(id, find_room_max));
        }
        try pair.expectSamePool();
    }
}

test "getNodeAtIdx and getNodeIdx: every slot has the original's number" {
    const gpa = testing.allocator;
    for (pool_sizes) |size| {
        errdefer std.debug.print(
            "  {d} nodes, {d} buckets\n",
            .{ size.node_count_max, size.hash_size },
        );
        var pair = try PoolPair.init(gpa, size.node_count_max, size.hash_size);
        defer pair.deinit(gpa);

        // Number zero is no node, and no node is number zero.
        try testing.expect(reference.ref_node_pool_get_node_at_idx(pair.reference_pool, 0) == null);
        try testing.expect(pair.pool.getNodeAtIdx(0) == null);
        try testing.expectEqual(
            @as(u32, 0),
            reference.ref_node_pool_get_node_idx(pair.reference_pool, null),
        );
        try testing.expectEqual(@as(u32, 0), pair.pool.getNodeIdx(null));
        try expectNoSlot(try pair.expectSameNode(null, null));

        // Every number up to the pool's size names a slot, handed out or
        // not. A number above that is outside the original's array, and
        // neither side is asked for it.
        var idx: u32 = 1;
        while (idx <= size.node_count_max) : (idx += 1) {
            errdefer std.debug.print("  number {d}\n", .{idx});
            const original = reference.ref_node_pool_get_node_at_idx(pair.reference_pool, idx);
            const port = pair.pool.getNodeAtIdx(idx);
            try expectSlot(idx - 1, try pair.expectSameNode(original, port));
        }

        // The numbers stay the same once nodes are handed out.
        const take_count = @min(size.takeCount(), 8);
        for (0..take_count) |taken| {
            const slot = try pair.getNode(spreadId(@intCast(taken)), 0);
            try expectSlot(@intCast(taken), slot);
            const original = reference.ref_node_pool_get_node_at_idx(
                pair.reference_pool,
                @intCast(taken + 1),
            );
            const port = pair.pool.getNodeAtIdx(@intCast(taken + 1));
            try expectSlot(@intCast(taken), try pair.expectSameNode(original, port));
        }
    }
}

test "getNode: a pool hands its slots out in order and none past the last" {
    const gpa = testing.allocator;
    // No pool here has room for this many nodes, so no node took this id.
    const unseen_id = spreadId(0x20000);

    for (pool_sizes) |size| {
        errdefer std.debug.print(
            "  {d} nodes, {d} buckets\n",
            .{ size.node_count_max, size.hash_size },
        );
        var pair = try PoolPair.init(gpa, size.node_count_max, size.hash_size);
        defer pair.deinit(gpa);

        const take_count = size.takeCount();
        var taken: u32 = 0;
        while (taken < take_count) : (taken += 1) {
            const slot = try pair.getNode(spreadId(taken), @intCast(taken & 3));
            try expectSlot(taken, slot);
        }
        try testing.expectEqual(@as(i32, @intCast(take_count)), pair.pool.getNodeCount());
        try pair.expectSamePool();
        if (take_count < size.node_count_max) continue;

        // A full pool has no node for a polygon it has not seen, nor for
        // another state of one it has.
        for (0..4) |state| try expectNoSlot(try pair.getNode(unseen_id, @intCast(state)));
        try expectNoSlot(try pair.getNode(spreadId(0), 1));
        try expectNoSlot(try pair.findNode(unseen_id, 0));
        try testing.expectEqual(@as(i32, @intCast(take_count)), pair.pool.getNodeCount());

        // The nodes it holds are still handed back.
        for ([_]u32{ 0, take_count / 2, take_count - 1 }) |held| {
            try expectSlot(held, try pair.getNode(spreadId(held), @intCast(held & 3)));
            try expectSlot(held, try pair.findNode(spreadId(held), @intCast(held & 3)));
        }
        try pair.expectSamePool();

        // A cleared pool has forgotten every node and starts again at the
        // first slot, whose position is the one it held before.
        pair.clear();
        try testing.expectEqual(@as(i32, 0), pair.pool.getNodeCount());
        try pair.expectSamePool();
        try expectNoSlot(try pair.findNode(spreadId(0), 0));
        try testing.expectEqual(@as(usize, 0), try pair.findNodes(spreadId(0), find_room_max));
        try expectSlot(0, try pair.getNode(unseen_id, 2));
        try expectSlot(0, try pair.findNode(unseen_id, 2));
        try testing.expectEqual(
            NodeFields.ofSlot(0).pos,
            NodeFields.ofPort(&pair.pool.nodes[0]).pos,
        );
        try pair.expectSamePool();
    }
}

test "getNode and findNode: each state of a polygon has its own node" {
    const gpa = testing.allocator;
    var pair = try PoolPair.init(gpa, 32, 4);
    defer pair.deinit(gpa);

    const ids = [_]PolyRef{ 0, 1, 2, 77, std.math.maxInt(PolyRef) };
    var taken: u32 = 0;
    for (ids) |id| {
        for (0..4) |state_index| {
            const state: u8 = @intCast(state_index);
            // Not there before it is taken.
            try expectNoSlot(try pair.findNode(id, state));
            try expectSlot(taken, try pair.getNode(id, state));
            taken += 1;
            // Asking again gives the same node and takes no other.
            try expectSlot(taken - 1, try pair.getNode(id, state));
            try expectSlot(taken - 1, try pair.findNode(id, state));
            try testing.expectEqual(@as(i32, @intCast(taken)), pair.pool.getNodeCount());
        }
    }

    // Every polygon and state still names its own slot.
    var expected: u32 = 0;
    for (ids) |id| {
        for (0..4) |state| {
            try expectSlot(expected, try pair.findNode(id, @intCast(state)));
            expected += 1;
        }
    }
    try pair.expectSamePool();

    // A node handed back a second time keeps what a search wrote into it.
    const slot = 6;
    var written = NodeFields.ofPort(&pair.pool.nodes[slot]);
    written.pos = .{ 1.5, -2.25, 8.0 };
    written.cost = 3.5;
    written.total = 9.25;
    written.parent_index = 3;
    written.flags = reference.ref_const_node_open();
    pair.writeNode(slot, written);
    try expectSlot(slot, try pair.getNode(ids[1], 2));
    try written.expectSame(NodeFields.ofPort(&pair.pool.nodes[slot]));
    try written.expectSame(NodeFields.ofReference(pair.referenceNode(slot)));

    // A node taken anew starts with zero costs, no parent and no flags,
    // and with the position its slot held.
    try expectSlot(taken, try pair.getNode(1000, 1));
    const fresh = NodeFields.ofPort(&pair.pool.nodes[taken]);
    try testing.expectEqual(@as(f32, 0), fresh.cost);
    try testing.expectEqual(@as(f32, 0), fresh.total);
    try testing.expectEqual(@as(u32, 0), fresh.parent_index);
    try testing.expectEqual(@as(u32, 1), fresh.state);
    try testing.expectEqual(@as(u32, 0), fresh.flags);
    try testing.expectEqual(@as(PolyRef, 1000), fresh.id);
    try testing.expectEqual(NodeFields.ofSlot(taken).pos, fresh.pos);
}

test "getNode: a state of four or more takes another node on every call" {
    const gpa = testing.allocator;
    const node_count_max = 16;
    var pair = try PoolPair.init(gpa, node_count_max, 2);
    defer pair.deinit(gpa);

    const id: PolyRef = 99;
    var taken: u32 = 0;
    for ([_]u8{ 4, 5, 7, 255, 4, 4 }) |state| {
        errdefer std.debug.print("  state {d}\n", .{state});
        try expectSlot(taken, try pair.getNode(id, state));
        // The node keeps the low bits of the state, and no lookup finds it
        // under the state it was taken with.
        const stored = NodeFields.ofPort(&pair.pool.nodes[taken]).state;
        try testing.expectEqual(@as(u32, state & 3), stored);
        try expectNoSlot(try pair.findNode(id, state));
        taken += 1;
    }

    // The low bits are states of their own: the last node taken with each
    // is the one a lookup finds, and none was taken with the bits of two.
    try expectSlot(5, try pair.findNode(id, 0));
    try expectSlot(1, try pair.findNode(id, 1));
    try expectNoSlot(try pair.findNode(id, 2));
    try expectSlot(3, try pair.findNode(id, 3));
    try expectSlot(5, try pair.getNode(id, 0));

    // One polygon now has more nodes than it has states.
    try testing.expectEqual(@as(usize, 6), try pair.findNodes(id, find_room_max));
    try testing.expectEqual(@as(usize, 4), try pair.findNodes(id, 4));
    try testing.expectEqual(@as(usize, 0), try pair.findNodes(id, 0));

    while (taken < node_count_max) : (taken += 1) {
        try expectSlot(taken, try pair.getNode(id, 6));
    }
    try expectNoSlot(try pair.getNode(id, 6));
    try expectNoSlot(try pair.getNode(id, 4));
    try expectSlot(node_count_max - 1, try pair.getNode(id, 2));
    try testing.expectEqual(@as(usize, find_room_max), try pair.findNodes(id, find_room_max));
    try pair.expectSamePool();
}

test "findNodes: every buffer size gets the original's nodes in the original's order" {
    const gpa = testing.allocator;
    var pair = try PoolPair.init(gpa, 64, 4);
    defer pair.deinit(gpa);

    // Polygons with four nodes, two, one and none, taken between nodes of
    // other polygons that land in the same four buckets.
    const four = spreadId(1);
    const two = spreadId(2);
    const one = spreadId(3);
    const none = spreadId(4);
    var filler: u32 = 100;
    for (0..4) |state| {
        _ = try pair.getNode(four, @intCast(state));
        _ = try pair.getNode(spreadId(filler), @intCast(state));
        filler += 1;
    }
    _ = try pair.getNode(two, 1);
    _ = try pair.getNode(spreadId(filler), 0);
    _ = try pair.getNode(two, 3);
    _ = try pair.getNode(one, 2);
    for (0..12) |_| {
        filler += 1;
        _ = try pair.getNode(spreadId(filler), 0);
    }
    try testing.expectEqual(@as(i32, 24), pair.pool.getNodeCount());

    const cases = [_]struct { id: PolyRef, count: usize }{
        .{ .id = four, .count = 4 },
        .{ .id = two, .count = 2 },
        .{ .id = one, .count = 1 },
        .{ .id = none, .count = 0 },
        .{ .id = spreadId(100), .count = 1 },
        .{ .id = 0, .count = 0 },
    };
    for (cases) |case| {
        for ([_]usize{ 0, 1, 2, 3, 4, find_room_max }) |room| {
            // A buffer too small gets the first nodes of the chain and the
            // count it has room for.
            const count = try pair.findNodes(case.id, room);
            try testing.expectEqual(@min(room, case.count), count);
        }
    }
    try pair.expectSamePool();

    pair.clear();
    for (cases) |case| {
        try testing.expectEqual(@as(usize, 0), try pair.findNodes(case.id, find_room_max));
    }
}

test "NodePool: a pool with no buckets reports the original's sizes" {
    const gpa = testing.allocator;
    // Every lookup in such a pool reads a bucket it does not have, on both
    // sides, so only what leaves the buckets alone is compared.
    for ([_]i32{ 1, 3, 16 }) |node_count_max| {
        const reference_pool = reference.ref_node_pool_create(node_count_max, 0) orelse
            return error.OutOfMemory;
        defer reference.ref_node_pool_destroy(reference_pool);
        var pool: NodePool = .empty;
        try pool.init(gpa, node_count_max, 0);
        defer pool.deinit(gpa);

        for (0..2) |_| {
            try testing.expectEqual(
                reference.ref_node_pool_get_mem_used(reference_pool),
                pool.getMemUsed(),
            );
            try testing.expectEqual(
                reference.ref_node_pool_get_max_nodes(reference_pool),
                pool.getMaxNodes(),
            );
            try testing.expectEqual(
                reference.ref_node_pool_get_hash_size(reference_pool),
                pool.getHashSize(),
            );
            try testing.expectEqual(
                reference.ref_node_pool_get_node_count(reference_pool),
                pool.getNodeCount(),
            );
            try testing.expectEqual(
                reference.ref_node_pool_get_node_idx(reference_pool, null),
                pool.getNodeIdx(null),
            );
            try testing.expect(reference.ref_node_pool_get_node_at_idx(reference_pool, 0) == null);
            try testing.expect(pool.getNodeAtIdx(0) == null);

            reference.ref_node_pool_clear(reference_pool);
            pool.clear();
        }
    }
}

/// One pool the random test runs over, and how its polygon ids are drawn.
const PoolFuzz = struct {
    node_count_max: i32,
    hash_size: i32,
    /// How many ids the test keeps drawing. Few of them for the pool's size
    /// bring up every state of a polygon and fill the buckets' chains.
    id_count: u32,
    /// Out of a hundred draws, how many take any id there is instead of one
    /// of the `id_count`.
    any_id_percent: u32,
};

const pool_fuzzes = [_]PoolFuzz{
    .{ .node_count_max = 8, .hash_size = 1, .id_count = 3, .any_id_percent = 0 },
    .{ .node_count_max = 16, .hash_size = 4, .id_count = 6, .any_id_percent = 0 },
    .{ .node_count_max = 64, .hash_size = 8, .id_count = 24, .any_id_percent = 5 },
    .{ .node_count_max = 64, .hash_size = 64, .id_count = 12, .any_id_percent = 0 },
    .{ .node_count_max = 256, .hash_size = 64, .id_count = 64, .any_id_percent = 60 },
    .{ .node_count_max = 300, .hash_size = 16, .id_count = 64, .any_id_percent = 90 },
};

const pool_fuzz_seeds = [_]u64{ 1, 2, 0x706f6f6c, 0xdecafbad };

/// How many calls one seed makes on one pool.
const pool_fuzz_operation_count = 2000;

/// The most ids a `PoolFuzz` may keep drawing.
const fuzz_id_count_max = 64;

/// The buffer sizes the random test gives `findNodes`.
const find_rooms = [_]usize{ 0, 1, 2, 4, find_room_max };

/// Makes random calls on a pool of each side and compares what every call
/// hands back, and every fifty calls the whole of the two pools.
fn fuzzPool(gpa: std.mem.Allocator, fuzz: PoolFuzz, seed: u64) !void {
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(seed));
    const random = prng.random();

    var pair = try PoolPair.init(gpa, fuzz.node_count_max, fuzz.hash_size);
    defer pair.deinit(gpa);

    var ids: [fuzz_id_count_max]PolyRef = undefined;
    for (&ids) |*id| id.* = random.int(PolyRef);
    // Polygon zero is an id like any other to a pool.
    ids[0] = 0;

    const node_count_max: u32 = @intCast(fuzz.node_count_max);
    for (0..pool_fuzz_operation_count) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });

        const id = if (random.uintLessThan(u32, 100) < fuzz.any_id_percent)
            random.int(PolyRef)
        else
            ids[random.uintLessThan(u32, fuzz.id_count)];
        // Mostly the states a search uses, now and then one above them.
        const state: u8 = if (random.uintLessThan(u32, 10) == 0)
            random.int(u8)
        else
            random.int(u2);

        // A pool is cleared about once in three times its size in calls, so
        // it is full for a good part of the run.
        if (random.uintLessThan(u32, node_count_max * 3) == 0) {
            pair.clear();
        } else switch (random.uintLessThan(u32, 100)) {
            0...44 => _ = try pair.getNode(id, state),
            45...64 => _ = try pair.findNode(id, state),
            65...74 => {
                const room = find_rooms[random.uintLessThan(usize, find_rooms.len)];
                _ = try pair.findNodes(id, room);
            },
            75...89 => {
                // What a search does to a node it was handed: it writes
                // everything but the polygon and the state the pool knows
                // the node by.
                const node_count: u32 = @intCast(pair.pool.getNodeCount());
                if (node_count > 0) {
                    const slot = random.uintLessThan(u32, node_count);
                    const kept = NodeFields.ofPort(&pair.pool.nodes[slot]);
                    var fields: NodeFields = .random(random);
                    fields.id = kept.id;
                    fields.state = kept.state;
                    pair.writeNode(slot, fields);
                }
            },
            else => {
                const idx = random.uintAtMost(u32, node_count_max);
                const original = reference.ref_node_pool_get_node_at_idx(pair.reference_pool, idx);
                const port = pair.pool.getNodeAtIdx(idx);
                _ = try pair.expectSameNode(original, port);
            },
        }

        try pair.expectSameCount();
        if (case % 50 == 49) try pair.expectSamePool();
    }
    try pair.expectSamePool();
}

test "NodePool: random calls give the original's nodes, fields and bucket chains" {
    const gpa = testing.allocator;
    for (pool_fuzzes) |fuzz| {
        try testing.expect(fuzz.id_count <= fuzz_id_count_max);
        for (pool_fuzz_seeds) |seed| {
            errdefer std.debug.print(
                "  {d} nodes, {d} buckets, {d} ids\n",
                .{ fuzz.node_count_max, fuzz.hash_size, fuzz.id_count },
            );
            try fuzzPool(gpa, fuzz, seed);
        }
    }
}

// ---------------------------------------------------------------------------
// Node queue
// ---------------------------------------------------------------------------

/// One node queue on each side over the same nodes, given the same calls.
const QueuePair = struct {
    queue: NodeQueue,
    reference_queue: *reference.NodeQueue,
    /// The port's nodes. Node `i` here and node `i` of the original's are
    /// the same node to the tests.
    nodes: []Node,
    /// The original's nodes, `ref_sizeof_node()` bytes each.
    reference_nodes: []align(8) u8,
    /// Which nodes are in the queue. The tests push no node twice, never
    /// more nodes than the queue holds, and pop no empty queue: the original
    /// checks none of it.
    queued: []bool,
    queued_count: usize,

    fn init(gpa: std.mem.Allocator, capacity: i32, node_count: usize) !QueuePair {
        const reference_queue = reference.ref_node_queue_create(capacity) orelse
            return error.OutOfMemory;
        errdefer reference.ref_node_queue_destroy(reference_queue);

        const nodes = try gpa.alloc(Node, node_count);
        errdefer gpa.free(nodes);
        @memset(nodes, std.mem.zeroes(Node));

        const reference_size = node_count * reference.ref_sizeof_node();
        const reference_nodes = try gpa.alignedAlloc(u8, .@"8", reference_size);
        errdefer gpa.free(reference_nodes);
        @memset(reference_nodes, 0);

        const queued = try gpa.alloc(bool, node_count);
        errdefer gpa.free(queued);
        @memset(queued, false);

        var pair: QueuePair = .{
            .queue = .empty,
            .reference_queue = reference_queue,
            .nodes = nodes,
            .reference_nodes = reference_nodes,
            .queued = queued,
            .queued_count = 0,
        };
        try pair.queue.init(gpa, capacity);
        for (0..node_count) |i| pair.setTotal(i, 0);
        return pair;
    }

    fn deinit(pair: *QueuePair, gpa: std.mem.Allocator) void {
        pair.queue.deinit(gpa);
        reference.ref_node_queue_destroy(pair.reference_queue);
        gpa.free(pair.queued);
        gpa.free(pair.reference_nodes);
        gpa.free(pair.nodes);
    }

    fn referenceNode(pair: *const QueuePair, i: usize) *reference.Node {
        return @ptrCast(&pair.reference_nodes[i * reference.ref_sizeof_node()]);
    }

    /// Which of the original's nodes a call handed back, -1 for none.
    fn referenceIndex(pair: *const QueuePair, node: ?*const reference.Node) i64 {
        const found = node orelse return -1;
        const offset = @intFromPtr(found) -% @intFromPtr(pair.reference_nodes.ptr);
        return @intCast(offset / reference.ref_sizeof_node());
    }

    /// Which of the port's nodes a call handed back.
    fn portIndex(pair: *const QueuePair, node: *const Node) i64 {
        const offset = @intFromPtr(node) -% @intFromPtr(pair.nodes.ptr);
        return @intCast(offset / @sizeOf(Node));
    }

    /// Gives node `i` a total on both sides. A queue reads nothing else of
    /// a node, and the rest is the same on both sides anyway.
    fn setTotal(pair: *QueuePair, i: usize, total: f32) void {
        const fields: NodeFields = .{
            .pos = .{ 0, 0, 0 },
            .cost = 0,
            .total = total,
            .parent_index = 0,
            .state = 0,
            .flags = 0,
            .id = @intCast(i + 1),
        };
        fields.writeReference(pair.referenceNode(i));
        fields.writePort(&pair.nodes[i]);
    }

    /// Pushes node `i`, which must not be queued, with the total it has.
    fn push(pair: *QueuePair, i: usize) !void {
        try testing.expect(!pair.queued[i]);
        try testing.expect(pair.queued_count < pair.queue.getCapacity());
        reference.ref_node_queue_push(pair.reference_queue, pair.referenceNode(i));
        pair.queue.push(&pair.nodes[i]);
        pair.queued[i] = true;
        pair.queued_count += 1;
    }

    /// Tells both queues that node `i` has a new total.
    fn modify(pair: *QueuePair, i: usize) void {
        reference.ref_node_queue_modify(pair.reference_queue, pair.referenceNode(i));
        pair.queue.modify(&pair.nodes[i]);
    }

    /// The node on top of both queues, which must not be empty.
    fn top(pair: *QueuePair) !usize {
        try testing.expect(pair.queued_count > 0);
        const original = pair.referenceIndex(reference.ref_node_queue_top(pair.reference_queue));
        const port = pair.portIndex(pair.queue.top());
        if (original != port) {
            std.debug.print("  top(): original node {d}, port node {d}\n", .{ original, port });
            return error.TestExpectedEqual;
        }
        return @intCast(port);
    }

    /// Pops both queues, which must not be empty; the node both gave.
    fn pop(pair: *QueuePair) !usize {
        try testing.expect(pair.queued_count > 0);
        const original = pair.referenceIndex(reference.ref_node_queue_pop(pair.reference_queue));
        const port = pair.portIndex(pair.queue.pop());
        if (original != port) {
            std.debug.print("  pop(): original node {d}, port node {d}\n", .{ original, port });
            return error.TestExpectedEqual;
        }
        const i: usize = @intCast(port);
        try testing.expect(pair.queued[i]);
        pair.queued[i] = false;
        pair.queued_count -= 1;
        return i;
    }

    fn clear(pair: *QueuePair) void {
        reference.ref_node_queue_clear(pair.reference_queue);
        pair.queue.clear();
        @memset(pair.queued, false);
        pair.queued_count = 0;
    }

    /// Fails unless both queues say the same about being empty, and what
    /// the calls so far make true, have the same sizes and, when they hold
    /// nodes, the same node on top.
    fn expectSameState(pair: *QueuePair) !void {
        const original_empty = reference.ref_node_queue_empty(pair.reference_queue);
        const port_empty = pair.queue.isEmpty();
        if (original_empty != port_empty) {
            std.debug.print(
                "  isEmpty(): original {}, port {}\n",
                .{ original_empty, port_empty },
            );
            return error.TestExpectedEqual;
        }
        try testing.expectEqual(pair.queued_count == 0, port_empty);
        try testing.expectEqual(
            reference.ref_node_queue_get_capacity(pair.reference_queue),
            pair.queue.getCapacity(),
        );
        try testing.expectEqual(
            reference.ref_node_queue_get_mem_used(pair.reference_queue),
            pair.queue.getMemUsed(),
        );
        if (!port_empty) _ = try pair.top();
    }

    /// Pops both queues empty and fails unless they give the same nodes in
    /// the same order. The nodes are written to `order` as they come out;
    /// returns how many there were.
    fn drain(pair: *QueuePair, order: []usize) !usize {
        var count: usize = 0;
        while (pair.queued_count > 0) : (count += 1) {
            order[count] = try pair.pop();
            try pair.expectSameState();
        }
        return count;
    }

    /// A node that is in the queue (`wanted` true) or out of it, picked at
    /// random; null when there is none.
    fn pick(pair: *const QueuePair, random: std.Random, wanted: bool) ?usize {
        const start = random.uintLessThan(usize, pair.queued.len);
        for (0..pair.queued.len) |step| {
            const i = (start + step) % pair.queued.len;
            if (pair.queued[i] == wanted) return i;
        }
        return null;
    }
};

test "NodeQueue.init, getCapacity, getMemUsed and isEmpty: a new queue is empty" {
    const gpa = testing.allocator;
    for ([_]i32{ 1, 2, 3, 4, 7, 8, 64, 1000, 65535, 1 << 16 }) |capacity| {
        errdefer std.debug.print("  capacity {d}\n", .{capacity});
        var pair = try QueuePair.init(gpa, capacity, 1);
        defer pair.deinit(gpa);

        try pair.expectSameState();
        try testing.expect(pair.queue.isEmpty());
        try testing.expectEqual(capacity, pair.queue.getCapacity());

        // What is left of the size with the heap taken off, one pointer more
        // than the capacity, is the original's queue object itself.
        const heap = @sizeOf(usize) * (@as(i64, capacity) + 1);
        try testing.expectEqual(
            @as(i64, @intCast(reference.ref_sizeof_node_queue())),
            pair.queue.getMemUsed() - heap,
        );

        // Clearing an empty queue leaves it empty.
        pair.clear();
        try pair.expectSameState();
    }
}

test "push, top and pop: nodes leave lowest total first" {
    const gpa = testing.allocator;
    const totals = [_]f32{ 40, 7.5, 93, -12, 0.25, 61, 18, 5, 77, -3, 33, 2, 99, 50, 11, 64 };
    var pair = try QueuePair.init(gpa, totals.len, totals.len);
    defer pair.deinit(gpa);

    var lowest = std.math.inf(f32);
    for (totals, 0..) |total, i| {
        pair.setTotal(i, total);
        try pair.push(i);
        try pair.expectSameState();
        // The top is the lowest total pushed so far.
        lowest = @min(lowest, total);
        try testing.expectEqual(lowest, totals[try pair.top()]);
    }

    // Looking at the top takes nothing out.
    const first = try pair.top();
    try testing.expectEqual(first, try pair.top());
    try testing.expectEqual(@as(usize, totals.len), pair.queued_count);

    var order: [totals.len]usize = undefined;
    try testing.expectEqual(@as(usize, totals.len), try pair.drain(&order));
    try testing.expectEqual(first, order[0]);
    for (order[0 .. order.len - 1], order[1..]) |earlier, later| {
        try testing.expect(totals[earlier] < totals[later]);
    }
    try testing.expect(pair.queue.isEmpty());
}

test "push and pop: equal totals leave in the original's order" {
    const gpa = testing.allocator;
    // Among equal totals the order is whatever the heap's moves make it:
    // the place where two heaps that both sort correctly would differ.
    const totals = [_]f32{ 1, 1, 1, 2, 2, 0, 0, 0, 0, 5, 5, 1, 2, 0, 5, 1, 1, 2, 2, 0 };
    var pair = try QueuePair.init(gpa, totals.len, totals.len);
    defer pair.deinit(gpa);
    var order: [totals.len]usize = undefined;

    for (totals, 0..) |total, i| {
        pair.setTotal(i, total);
        try pair.push(i);
        try pair.expectSameState();
    }
    try testing.expectEqual(@as(usize, totals.len), try pair.drain(&order));
    for (order[0 .. order.len - 1], order[1..]) |earlier, later| {
        try testing.expect(totals[earlier] <= totals[later]);
    }

    // The same with pops between the pushes.
    for (totals, 0..) |_, i| {
        try pair.push(i);
        try pair.expectSameState();
        if (i % 3 == 2) {
            _ = try pair.pop();
            try pair.expectSameState();
        }
    }
    _ = try pair.drain(&order);

    // Every node with one total.
    for (totals, 0..) |_, i| {
        pair.setTotal(i, 4.5);
        try pair.push(i);
    }
    try testing.expectEqual(@as(usize, totals.len), try pair.drain(&order));
}

test "modify: a lowered total moves a node up, a raised one leaves it where it was" {
    const gpa = testing.allocator;
    var pair = try QueuePair.init(gpa, 8, 10);
    defer pair.deinit(gpa);
    var order: [10]usize = undefined;

    for (0..8) |i| {
        pair.setTotal(i, @floatFromInt((i + 1) * 10));
        try pair.push(i);
    }
    try testing.expectEqual(@as(usize, 0), try pair.top());

    // The last node pushed, at the bottom of the heap, gets the lowest
    // total and climbs to the top.
    pair.setTotal(7, 5);
    pair.modify(7);
    try pair.expectSameState();
    try testing.expectEqual(@as(usize, 7), try pair.top());

    // A node in the middle gets a total between two others.
    pair.setTotal(5, 15);
    pair.modify(5);
    try pair.expectSameState();

    // A total that stays the same moves nothing.
    pair.modify(3);
    try pair.expectSameState();

    // A raised total moves nothing either: a queue only ever moves a node
    // up, so the top stays on top with the highest total there is.
    pair.setTotal(7, 1000);
    pair.modify(7);
    try pair.expectSameState();
    try testing.expectEqual(@as(usize, 7), try pair.top());

    // Nodes that are not in the queue are left alone, with any total.
    pair.setTotal(8, -50);
    pair.modify(8);
    pair.setTotal(9, 50);
    pair.modify(9);
    try pair.expectSameState();
    try testing.expectEqual(@as(usize, 8), pair.queued_count);

    try testing.expectEqual(@as(usize, 8), try pair.drain(&order));
    try testing.expectEqual(@as(usize, 7), order[0]);

    // Lowering every node in turn below all the others turns the order
    // around.
    for (0..8) |i| {
        pair.setTotal(i, @floatFromInt(100 + i));
        try pair.push(i);
    }
    for (0..8) |i| {
        pair.setTotal(i, 50 - @as(f32, @floatFromInt(i)));
        pair.modify(i);
        try pair.expectSameState();
        try testing.expectEqual(i, try pair.top());
    }
    try testing.expectEqual(@as(usize, 8), try pair.drain(&order));
    for (order[0..8], 0..) |node, place| try testing.expectEqual(7 - place, node);
}

test "push, top, pop, modify and clear: a queue of one node" {
    const gpa = testing.allocator;
    var pair = try QueuePair.init(gpa, 1, 3);
    defer pair.deinit(gpa);

    pair.setTotal(0, 3);
    try pair.push(0);
    try pair.expectSameState();
    try testing.expectEqual(@as(usize, 0), try pair.top());
    pair.modify(0);
    pair.modify(1);
    try pair.expectSameState();
    try testing.expectEqual(@as(usize, 0), try pair.pop());
    try pair.expectSameState();
    try testing.expect(pair.queue.isEmpty());

    pair.setTotal(1, -std.math.inf(f32));
    try pair.push(1);
    try pair.expectSameState();
    pair.setTotal(1, std.math.inf(f32));
    pair.modify(1);
    try testing.expectEqual(@as(usize, 1), try pair.top());

    pair.clear();
    try pair.expectSameState();
    try testing.expect(pair.queue.isEmpty());

    // A cleared queue takes nodes again.
    try pair.push(2);
    try testing.expectEqual(@as(usize, 2), try pair.pop());
    try pair.expectSameState();
}

test "push: a queue filled to its capacity pops every node" {
    const gpa = testing.allocator;
    for ([_]i32{ 2, 3, 7, 8, 9, 100, 1000 }) |capacity| {
        errdefer std.debug.print("  capacity {d}\n", .{capacity});
        const node_count: usize = @intCast(capacity);
        var pair = try QueuePair.init(gpa, capacity, node_count);
        defer pair.deinit(gpa);
        const order = try gpa.alloc(usize, node_count);
        defer gpa.free(order);

        // Falling totals: every push climbs the whole heap.
        for (0..node_count) |i| {
            pair.setTotal(i, @floatFromInt(node_count - i));
            try pair.push(i);
            try testing.expectEqual(i, try pair.top());
        }
        try pair.expectSameState();
        try testing.expectEqual(node_count, try pair.drain(order));
        for (order, 0..) |node, place| try testing.expectEqual(node_count - 1 - place, node);

        // Rising totals: no push moves anything, every pop sinks a node.
        for (0..node_count) |i| {
            pair.setTotal(i, @floatFromInt(i));
            try pair.push(i);
        }
        try pair.expectSameState();
        try testing.expectEqual(node_count, try pair.drain(order));
        for (order, 0..) |node, place| try testing.expectEqual(place, node);

        // A full queue cleared is empty and fills again.
        for (0..node_count) |i| try pair.push(i);
        pair.clear();
        try pair.expectSameState();
        for (0..node_count) |i| try pair.push(node_count - 1 - i);
        try pair.expectSameState();
        try testing.expectEqual(node_count, try pair.drain(order));
    }
}

test "push and pop: zeroes of both signs, infinities and totals that are not numbers" {
    const gpa = testing.allocator;
    const inf = std.math.inf(f32);
    const nan = std.math.nan(f32);
    // A total that is not a number compares false with everything on both
    // sides, so the heap's moves around it are still the same moves.
    const totals = [_]f32{
        0.0,  -0.0,                   inf,                     -inf,
        nan,  1.0,                    -1.0,                    nan,
        inf,  -inf,                   0.0,                     -0.0,
        -nan, std.math.floatMax(f32), -std.math.floatMax(f32), std.math.floatTrueMin(f32),
        nan,  -0.0,                   0.0,                     inf,
    };
    var pair = try QueuePair.init(gpa, totals.len, totals.len);
    defer pair.deinit(gpa);
    var order: [totals.len]usize = undefined;

    // Each stride shares no factor with the number of totals, so a round
    // pushes every node once, each round in another order.
    for ([_]usize{ 1, 3, 7 }) |stride| {
        for (0..totals.len) |step| {
            const i = (step * stride + stride) % totals.len;
            pair.setTotal(i, totals[i]);
            try pair.push(i);
            try pair.expectSameState();
        }
        // Raise and lower nodes to and from the special totals.
        pair.setTotal(4, -inf);
        pair.modify(4);
        try pair.expectSameState();
        pair.setTotal(9, nan);
        pair.modify(9);
        try pair.expectSameState();
        pair.setTotal(2, -0.0);
        pair.modify(2);
        try pair.expectSameState();
        try testing.expectEqual(@as(usize, totals.len), try pair.drain(&order));
    }
}

/// One queue the random test runs over.
const QueueFuzz = struct {
    capacity: i32,
    /// How many nodes the test has to push: more than the capacity, so
    /// some are always out of the queue.
    node_count: usize,
    /// Whether totals that are not numbers are drawn too. Without them
    /// only a raised total can break the heap's order.
    not_a_number: bool,
};

const queue_fuzzes = [_]QueueFuzz{
    .{ .capacity = 1, .node_count = 3, .not_a_number = true },
    .{ .capacity = 2, .node_count = 4, .not_a_number = false },
    .{ .capacity = 5, .node_count = 9, .not_a_number = true },
    .{ .capacity = 32, .node_count = 48, .not_a_number = false },
    .{ .capacity = 33, .node_count = 40, .not_a_number = true },
    .{ .capacity = 256, .node_count = 300, .not_a_number = false },
};

const queue_fuzz_seeds = [_]u64{ 3, 4, 0x71756575, 0xfeedface };

/// How many calls one seed makes on one queue.
const queue_fuzz_operation_count = 2500;

/// A total for a node: few different values, so equal totals come up all
/// the time, among them zeroes of both signs and the infinities.
fn randomTotal(random: std.Random, not_a_number: bool) f32 {
    return switch (random.uintLessThan(u32, 16)) {
        0 => 0.0,
        1 => -0.0,
        2 => std.math.inf(f32),
        3 => -std.math.inf(f32),
        4 => if (not_a_number) std.math.nan(f32) else 1.0,
        5, 6, 7, 8 => @as(f32, @floatFromInt(random.intRangeAtMost(i32, -3, 3))),
        9, 10 => 1.0,
        else => (random.float(f32) - 0.25) * 1000.0,
    };
}

/// Makes random calls on a queue of each side and compares, after every
/// call, the node handed back, whether the queue is empty and its top, and
/// now and then and at the end the order every node leaves in.
fn fuzzQueue(gpa: std.mem.Allocator, fuzz: QueueFuzz, seed: u64) !void {
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(seed));
    const random = prng.random();

    var pair = try QueuePair.init(gpa, fuzz.capacity, fuzz.node_count);
    defer pair.deinit(gpa);
    const order = try gpa.alloc(usize, fuzz.node_count);
    defer gpa.free(order);
    const capacity: usize = @intCast(fuzz.capacity);

    for (0..queue_fuzz_operation_count) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });

        const roll = random.uintLessThan(u32, 100);
        if (roll < 40) {
            // A full queue takes no push: it is popped instead.
            if (pair.queued_count < capacity) {
                const i = pair.pick(random, false) orelse return error.TestUnexpectedResult;
                pair.setTotal(i, randomTotal(random, fuzz.not_a_number));
                try pair.push(i);
            } else {
                _ = try pair.pop();
            }
        } else if (roll < 62) {
            if (pair.queued_count > 0) _ = try pair.pop();
        } else if (roll < 84) {
            // A queued node gets a lower total, as a search gives it, a
            // higher one, its own again or any total at all.
            if (pair.pick(random, true)) |i| {
                const total = pair.nodes[i].total;
                const step: f32 = switch (random.uintLessThan(u32, 4)) {
                    0 => 0.5,
                    1 => 1.0,
                    2 => 100.0,
                    else => random.float(f32) * 10.0,
                };
                const new_total = switch (random.uintLessThan(u32, 8)) {
                    0, 1, 2, 3 => total - step,
                    4, 5 => total + step,
                    6 => total,
                    else => randomTotal(random, fuzz.not_a_number),
                };
                pair.setTotal(i, new_total);
                pair.modify(i);
            }
        } else if (roll < 90) {
            // A node out of the queue: there is nothing to move.
            if (pair.pick(random, false)) |i| {
                pair.setTotal(i, randomTotal(random, fuzz.not_a_number));
                pair.modify(i);
            }
        } else if (roll < 92) {
            pair.clear();
        } else if (roll < 96) {
            // Empty both queues to compare the whole order, then push the
            // nodes back in the order they left.
            const count = try pair.drain(order);
            for (order[0..count]) |i| try pair.push(i);
        } else if (pair.queued_count > 0) {
            // Looking at the top twice gives the same node.
            const first = try pair.top();
            try testing.expectEqual(first, try pair.top());
        }
        try pair.expectSameState();
    }

    // The order the nodes still queued leave in.
    _ = try pair.drain(order);
    try testing.expect(pair.queue.isEmpty());
}

test "NodeQueue: random calls give the original's nodes in the original's order" {
    const gpa = testing.allocator;
    for (queue_fuzzes) |fuzz| {
        try testing.expect(fuzz.node_count > fuzz.capacity);
        for (queue_fuzz_seeds) |seed| {
            errdefer std.debug.print(
                "  capacity {d}, {d} nodes\n",
                .{ fuzz.capacity, fuzz.node_count },
            );
            try fuzzQueue(gpa, fuzz, seed);
        }
    }
}
