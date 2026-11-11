//! The search nodes of a navigation query: a fixed pool of nodes found by
//! polygon reference, and the open list that orders them by total cost.
//! Positions are whole millimetres and costs are in 1/256 of a millimetre.

const std = @import("std");
const common = @import("common.zig");
const nav_mesh = @import("nav_mesh.zig");

const PolyRef = nav_mesh.PolyRef;

/// A node's index in its pool's node array.
pub const NodeIndex = u16;

/// Ends a chain of node indices.
pub const null_index: NodeIndex = 0xffff;

/// How many bits of a node hold its parent's index.
pub const node_parent_bits = 24;

/// How many bits of a node hold its state.
pub const node_state_bits = 2;

/// How many nodes one polygon can have, one per state.
pub const node_states_max = 1 << node_state_bits;

/// Where a node stands in a search.
pub const NodeFlags = packed struct(u3) {
    /// The node waits in the open list.
    open: bool = false,
    /// The node has been expanded.
    closed: bool = false,
    /// The parent is not a neighbour: the step from it was found by a ray
    /// cast across several polygons.
    parent_detached: bool = false,
};

/// One polygon as a search reached it.
pub const Node = extern struct {
    /// The point the search stands on in the polygon, in millimetres.
    pos: [3]i32,
    /// The cost from the start to this node, in 1/256 of a millimetre.
    cost: u64,
    /// The cost plus the estimate of what is left to the goal, in 1/256 of
    /// a millimetre.
    total: u64,
    bits: Bits,
    /// The polygon the node stands for.
    id: PolyRef,

    /// The node's packed part.
    pub const Bits = packed struct(u32) {
        /// The parent as `NodePool.getNodeIdx` numbers it: zero for none.
        parent_index: u24 = 0,
        /// Tells apart the nodes of one polygon, such as by the side the
        /// search entered it from.
        state: u2 = 0,
        flags: NodeFlags = .{},
        reserved: u3 = 0,
    };
};

// What the pool and the queue themselves take on a 64-bit target, beside
// their arrays: three pointers and three integers, and one pointer and two
// integers.
const pool_header_size = 40;
const queue_header_size = 16;

fn hashRef(ref: PolyRef) u32 {
    var a = ref;
    if (nav_mesh.polyref64) {
        a = (~a) +% (a << 18);
        a ^= a >> 31;
        a *%= 21;
        a ^= a >> 11;
        a +%= a << 6;
        a ^= a >> 22;
    } else {
        a +%= ~(a << 15);
        a ^= a >> 10;
        a +%= a << 3;
        a ^= a >> 6;
        a +%= ~(a << 11);
        a ^= a >> 16;
    }
    return @truncate(a);
}

/// A fixed number of search nodes, found by polygon reference through a
/// hash table.
///
/// A pool starts as `empty`; `init` readies it and `deinit`, given the same
/// allocator, ends it. A `*Node` it hands out stays at its address until
/// `deinit`, but `clear` gives every node away again: a pointer or node
/// index kept across `clear` names whatever node is handed out there next.
pub const NodePool = struct {
    nodes: []Node,
    /// Per hash bucket, the bucket's first node or `null_index`.
    first: []NodeIndex,
    /// Per node, the next node of the same bucket or `null_index`.
    next: []NodeIndex,
    node_count_max: i32,
    hash_size: i32,
    node_count: i32,

    /// A pool with no nodes. Only `init` and `deinit` may be called on it.
    pub const empty: NodePool = .{
        .nodes = &.{},
        .first = &.{},
        .next = &.{},
        .node_count_max = 0,
        .hash_size = 0,
        .node_count = 0,
    };

    /// Readies an `empty` pool with room for `node_count_max` nodes, at most
    /// `null_index`, in `hash_size` buckets, a power of two. On failure the
    /// pool is left as it was.
    pub fn init(
        self: *NodePool,
        gpa: std.mem.Allocator,
        node_count_max: i32,
        hash_size: i32,
    ) error{OutOfMemory}!void {
        const hash_size_bits: u32 = @bitCast(hash_size);
        std.debug.assert(common.nextPow2(hash_size_bits) == hash_size_bits);
        std.debug.assert(node_count_max > 0 and node_count_max <= null_index and
            node_count_max <= (1 << node_parent_bits) - 1);

        const nodes = try gpa.alloc(Node, @intCast(node_count_max));
        errdefer gpa.free(nodes);
        const next = try gpa.alloc(NodeIndex, @intCast(node_count_max));
        errdefer gpa.free(next);
        const first = try gpa.alloc(NodeIndex, @intCast(hash_size));

        // A search reads a node's position before writing it when the edge
        // to the node cannot be found, so no node starts with stray bytes.
        @memset(nodes, std.mem.zeroes(Node));
        @memset(first, null_index);
        @memset(next, null_index);

        self.* = .{
            .nodes = nodes,
            .first = first,
            .next = next,
            .node_count_max = node_count_max,
            .hash_size = hash_size,
            .node_count = 0,
        };
    }

    /// Frees the pool's arrays with the allocator `init` was given. Every
    /// `*Node` from the pool is invalid afterwards.
    pub fn deinit(self: *NodePool, gpa: std.mem.Allocator) void {
        gpa.free(self.nodes);
        gpa.free(self.next);
        gpa.free(self.first);
        self.* = undefined;
    }

    /// Forgets every node. Pointers and node indices handed out before name
    /// other polygons' nodes once the pool fills again.
    pub fn clear(self: *NodePool) void {
        @memset(self.first, null_index);
        self.node_count = 0;
    }

    fn bucketOf(self: *const NodePool, id: PolyRef) u32 {
        return hashRef(id) & @as(u32, @bitCast(self.hash_size -% 1));
    }

    /// The node of polygon `id` in `state`, taking an unused node for it
    /// when there is none yet; null when the pool is full. A new node has
    /// no parent, no flags and zero costs, and its position is whatever the
    /// node last held. A `state` of `node_states_max` or above is never
    /// found again: every call takes another node for it, storing the
    /// state's low bits. The pointer is valid until `clear` or `deinit`.
    pub fn getNode(self: *NodePool, id: PolyRef, state: u8) ?*Node {
        const bucket = self.bucketOf(id);
        var i = self.first[bucket];
        while (i != null_index) : (i = self.next[i]) {
            const node = &self.nodes[i];
            if (node.id == id and node.bits.state == state) return node;
        }

        if (self.node_count >= self.node_count_max) return null;

        i = @intCast(self.node_count);
        self.node_count += 1;

        const node = &self.nodes[i];
        node.cost = 0;
        node.total = 0;
        node.id = id;
        node.bits = .{ .state = @truncate(state) };

        self.next[i] = self.first[bucket];
        self.first[bucket] = i;

        return node;
    }

    /// The node of polygon `id` in `state`, or null when the search has not
    /// reached it. The pointer is valid until `clear` or `deinit`.
    pub fn findNode(self: *const NodePool, id: PolyRef, state: u8) ?*Node {
        var i = self.first[self.bucketOf(id)];
        while (i != null_index) : (i = self.next[i]) {
            const node = &self.nodes[i];
            if (node.id == id and node.bits.state == state) return node;
        }
        return null;
    }

    /// Every node of polygon `id`, whatever its state, as many as `nodes`
    /// holds. Returns how many were written. The pointers are valid until
    /// `clear` or `deinit`.
    pub fn findNodes(self: *const NodePool, id: PolyRef, nodes: []*Node) usize {
        var n: usize = 0;
        var i = self.first[self.bucketOf(id)];
        while (i != null_index) : (i = self.next[i]) {
            if (self.nodes[i].id == id) {
                if (n >= nodes.len) return n;
                nodes[n] = &self.nodes[i];
                n += 1;
            }
        }
        return n;
    }

    /// A node's index plus one, the form `Node.Bits.parent_index` stores;
    /// zero for null. `node` must be one of this pool's.
    pub fn getNodeIdx(self: *const NodePool, node: ?*const Node) u32 {
        const n = node orelse return 0;
        const offset = @intFromPtr(n) - @intFromPtr(self.nodes.ptr);
        return @intCast(offset / @sizeOf(Node) + 1);
    }

    /// The node `getNodeIdx` numbered `idx`; null for zero. The pointer is
    /// valid until `clear` or `deinit`.
    pub fn getNodeAtIdx(self: *const NodePool, idx: u32) ?*Node {
        if (idx == 0) return null;
        return &self.nodes[idx - 1];
    }

    /// The bytes the pool and its arrays take on a 64-bit target.
    pub fn getMemUsed(self: *const NodePool) i32 {
        const node_count_max: u64 = @intCast(self.node_count_max);
        const hash_size: u64 = @intCast(self.hash_size);
        const total = pool_header_size +
            @sizeOf(Node) * node_count_max +
            @sizeOf(NodeIndex) * node_count_max +
            @sizeOf(NodeIndex) * hash_size;
        return @bitCast(@as(u32, @truncate(total)));
    }

    /// How many nodes the pool can hand out between two `clear` calls.
    pub fn getMaxNodes(self: *const NodePool) i32 {
        return self.node_count_max;
    }

    /// How many buckets the hash table has.
    pub fn getHashSize(self: *const NodePool) i32 {
        return self.hash_size;
    }

    /// The first node of a hash bucket, or `null_index`.
    pub fn getFirst(self: *const NodePool, bucket: i32) NodeIndex {
        return self.first[@intCast(bucket)];
    }

    /// The node after node `i` in its hash bucket, or `null_index`.
    pub fn getNext(self: *const NodePool, i: i32) NodeIndex {
        return self.next[@intCast(i)];
    }

    /// How many nodes are handed out.
    pub fn getNodeCount(self: *const NodePool) i32 {
        return self.node_count;
    }
};

/// A search's open list: a binary heap of nodes with the lowest `total` on
/// top.
///
/// A queue starts as `empty`; `init` readies it and `deinit`, given the
/// same allocator, ends it. It holds pointers into a `NodePool` and does
/// not check them: clear the queue whenever its pool is cleared or freed.
pub const NodeQueue = struct {
    /// One entry more than `capacity`.
    heap: []*Node,
    capacity: i32,
    size: i32,

    /// A queue with no heap. Only `init` and `deinit` may be called on it.
    pub const empty: NodeQueue = .{
        .heap = &.{},
        .capacity = 0,
        .size = 0,
    };

    /// Readies an `empty` queue for up to `capacity` nodes. On failure the
    /// queue is left as it was.
    pub fn init(
        self: *NodeQueue,
        gpa: std.mem.Allocator,
        capacity: i32,
    ) error{OutOfMemory}!void {
        std.debug.assert(capacity > 0);
        const heap = try gpa.alloc(*Node, @as(usize, @intCast(capacity)) + 1);
        self.* = .{
            .heap = heap,
            .capacity = capacity,
            .size = 0,
        };
    }

    /// Frees the heap with the allocator `init` was given.
    pub fn deinit(self: *NodeQueue, gpa: std.mem.Allocator) void {
        gpa.free(self.heap);
        self.* = undefined;
    }

    /// Empties the queue.
    pub fn clear(self: *NodeQueue) void {
        self.size = 0;
    }

    /// The node with the lowest total, left in the queue. The queue must
    /// not be empty.
    pub fn top(self: *NodeQueue) *Node {
        std.debug.assert(self.size > 0);
        return self.heap[0];
    }

    /// Takes the node with the lowest total out of the queue. The queue
    /// must not be empty.
    pub fn pop(self: *NodeQueue) *Node {
        std.debug.assert(self.size > 0);
        const result = self.heap[0];
        self.size -= 1;
        self.trickleDown(0, self.heap[@intCast(self.size)]);
        return result;
    }

    /// Adds a node that is not in the queue. The queue must hold fewer than
    /// `capacity` nodes.
    pub fn push(self: *NodeQueue, node: *Node) void {
        self.size += 1;
        self.bubbleUp(self.size - 1, node);
    }

    /// Moves a queued node up after its total went down. Does nothing for a
    /// node that is not in the queue.
    pub fn modify(self: *NodeQueue, node: *Node) void {
        var i: i32 = 0;
        while (i < self.size) : (i += 1) {
            if (self.heap[@intCast(i)] == node) {
                self.bubbleUp(i, node);
                return;
            }
        }
    }

    /// Whether the queue holds no node.
    pub fn isEmpty(self: *const NodeQueue) bool {
        return self.size == 0;
    }

    /// The bytes the queue and its heap take on a 64-bit target.
    pub fn getMemUsed(self: *const NodeQueue) i32 {
        const entry_count: u64 = @as(u64, @intCast(self.capacity)) + 1;
        const total = queue_header_size + 8 * entry_count;
        return @bitCast(@as(u32, @truncate(total)));
    }

    /// How many nodes the queue can hold.
    pub fn getCapacity(self: *const NodeQueue) i32 {
        return self.capacity;
    }

    fn bubbleUp(self: *NodeQueue, start: i32, node: *Node) void {
        var i = start;
        var parent = @divTrunc(i - 1, 2);
        // The parent of the top is the top itself: the index test ends the
        // climb there.
        while (i > 0 and self.heap[@intCast(parent)].total > node.total) {
            self.heap[@intCast(i)] = self.heap[@intCast(parent)];
            i = parent;
            parent = @divTrunc(i - 1, 2);
        }
        self.heap[@intCast(i)] = node;
    }

    fn trickleDown(self: *NodeQueue, start: i32, node: *Node) void {
        var i = start;
        var child = (i * 2) + 1;
        while (child < self.size) {
            if ((child + 1) < self.size and
                self.heap[@intCast(child)].total > self.heap[@intCast(child + 1)].total)
            {
                child += 1;
            }
            self.heap[@intCast(i)] = self.heap[@intCast(child)];
            i = child;
            child = (i * 2) + 1;
        }
        self.bubbleUp(i, node);
    }
};
