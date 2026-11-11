//! A queue of path requests served a few search iterations at a time, so
//! many agents asking for long paths share one bounded slice of each
//! update. The crowd owns one and polls it for results.

const std = @import("std");
const nav_mesh = @import("nav_mesh.zig");
const nav_mesh_query = @import("nav_mesh_query.zig");
const Status = @import("status.zig").Status;

const NavMesh = nav_mesh.NavMesh;
const PolyRef = nav_mesh.PolyRef;
const NavMeshQuery = nav_mesh_query.NavMeshQuery;
const QueryFilter = nav_mesh_query.QueryFilter;

/// A handle to a queued path request. It goes stale when the result is
/// taken with `getPathResult`, or when a finished request is left unread
/// for more than `PathQueue.keep_alive_max` updates: `getRequestStatus`
/// then fails for it. Handles are not reused until the counter wraps.
pub const PathQueueRef = u32;

/// The handle that names no request.
pub const path_queue_invalid: PathQueueRef = 0;

/// Path requests in a fixed number of entries, searched in turn by one
/// navigation query the queue owns.
///
/// A queue starts as `empty`; `init` readies it and `deinit` ends it with
/// the same allocator.
pub const PathQueue = struct {
    queue: [request_count_max]PathQuery,
    /// The handle the next request gets.
    next_handle: PathQueueRef,
    /// The length of every entry's path buffer.
    path_size_max: i32,
    /// Counts the entries `update` has moved past; its remainder by
    /// `request_count_max` is the entry served next.
    queue_head: i32,
    nav_query: ?*NavMeshQuery,

    /// The most requests the queue holds at once.
    pub const request_count_max = 8;
    /// How many updates a finished request waits to be read before its
    /// entry is given up.
    pub const keep_alive_max = 2;

    /// One request and its search result.
    const PathQuery = struct {
        /// The request's handle; `path_queue_invalid` marks a free entry.
        ref: PathQueueRef,
        /// Where the path starts and ends.
        start_pos: [3]f32,
        end_pos: [3]f32,
        start_ref: PolyRef,
        end_ref: PolyRef,
        /// The found path's buffer, `path_size_max` long; the first
        /// `path_count` are the result.
        path: []PolyRef,
        path_count: i32,
        /// All zero until the search starts, then the search's status.
        status: Status,
        /// The updates a finished request has waited to be read.
        keep_alive: i32,
        /// The requester's filter, which must outlive the request.
        filter: ?*const QueryFilter,

        const free: PathQuery = .{
            .ref = path_queue_invalid,
            .start_pos = .{ 0, 0, 0 },
            .end_pos = .{ 0, 0, 0 },
            .start_ref = 0,
            .end_ref = 0,
            .path = &.{},
            .path_count = 0,
            .status = .{},
            .keep_alive = 0,
            .filter = null,
        };
    };

    /// A queue that holds no memory and no requests. `request` and the
    /// lookups work on it; a request made before `init` must not reach
    /// `update`.
    pub const empty: PathQueue = .{
        .queue = @splat(PathQuery.free),
        .next_handle = 1,
        .path_size_max = 0,
        .queue_head = 0,
        .nav_query = null,
    };

    /// Frees the navigation query and every path buffer.
    fn purge(self: *PathQueue, gpa: std.mem.Allocator) void {
        if (self.nav_query) |nav_query| {
            nav_query.deinit(gpa);
            gpa.destroy(nav_query);
        }
        self.nav_query = null;
        for (&self.queue) |*q| {
            gpa.free(q.path);
            q.path = &.{};
            // No result is left to read once its buffer is gone.
            q.path_count = 0;
        }
    }

    /// Readies the queue for paths up to `path_size_max` polygons, searched
    /// over `nav` with a pool of `search_node_count_max` nodes. `nav` must
    /// outlive the queue.
    ///
    /// `self` is `empty` or a queue readied before with the same `gpa`,
    /// whose memory is freed first: every request it held is dropped, and
    /// the pointer `getNavQuery` gave is stale. Returns false when memory
    /// runs out or the query cannot be set up, leaving the queue holding
    /// no memory.
    pub fn init(
        self: *PathQueue,
        gpa: std.mem.Allocator,
        path_size_max: i32,
        search_node_count_max: i32,
        nav: *const NavMesh,
    ) bool {
        self.purge(gpa);

        const nav_query = gpa.create(NavMeshQuery) catch return false;
        nav_query.* = .empty;
        self.nav_query = nav_query;
        if (nav_query.init(gpa, nav, search_node_count_max).failed) {
            self.purge(gpa);
            return false;
        }

        self.path_size_max = path_size_max;
        for (&self.queue) |*q| {
            // An entry is free before its buffer is allocated, so the
            // requests of the entries after a failed one stay.
            q.ref = path_queue_invalid;
            // A negative size asks for more memory than there is.
            const path_size = std.math.cast(usize, path_size_max) orelse {
                self.purge(gpa);
                return false;
            };
            q.path = gpa.alloc(PolyRef, path_size) catch {
                self.purge(gpa);
                return false;
            };
        }

        self.queue_head = 0;

        return true;
    }

    /// Frees the queue's memory. `gpa` is the allocator `init` was given.
    pub fn deinit(self: *PathQueue, gpa: std.mem.Allocator) void {
        self.purge(gpa);
        self.* = undefined;
    }

    /// Advances the queued searches by up to `iteration_count_max` search
    /// iterations in all, taking the entries in turn from where the last
    /// update stopped, and ages the finished requests nobody has read.
    pub fn update(self: *PathQueue, iteration_count_max: i32) void {
        // Serve requests until none is left to serve or the iterations are
        // used up.
        var iteration_count = iteration_count_max;

        for (0..request_count_max) |_| {
            // The head counts on without end. Reading it as unsigned keeps
            // the rotation unbroken where the signed count wraps.
            const head: u32 = @bitCast(self.queue_head);
            const q = &self.queue[head % request_count_max];

            // Skip inactive requests.
            if (q.ref == path_queue_invalid) {
                self.queue_head +%= 1;
                continue;
            }

            // Handle a completed request.
            if (q.status.succeeded or q.status.failed) {
                // A result not read in a few updates gives up its entry.
                q.keep_alive += 1;
                if (q.keep_alive > keep_alive_max) {
                    q.ref = path_queue_invalid;
                    q.status = .{};
                }

                self.queue_head +%= 1;
                continue;
            }

            // Handle the start of a search.
            if (@as(u32, @bitCast(q.status)) == 0) {
                q.status = self.nav_query.?.initSlicedFindPath(
                    q.start_ref,
                    q.end_ref,
                    q.start_pos,
                    q.end_pos,
                    q.filter.?,
                    .{},
                );
            }
            // Handle a search in progress.
            if (q.status.in_progress) {
                var iterations: i32 = 0;
                q.status = self.nav_query.?.updateSlicedFindPath(iteration_count, &iterations);
                iteration_count -= iterations;
            }
            if (q.status.succeeded) {
                var path_count: usize = @intCast(q.path_count);
                q.status = self.nav_query.?.finalizeSlicedFindPath(q.path, &path_count);
                q.path_count = @intCast(path_count);
            }

            // Out of iterations: the head stays, so this entry is served
            // first next time.
            if (iteration_count <= 0) break;

            self.queue_head +%= 1;
        }
    }

    /// Queues a search for a path from `start_pos` on `start_ref` to
    /// `end_pos` on `end_ref`, and returns its handle, or
    /// `path_queue_invalid` when every entry is taken.
    ///
    /// The queue keeps the `filter` pointer and reads it in later `update`
    /// calls, so the filter must stay in place and unchanged until the
    /// request's result is taken or its handle goes stale.
    pub fn request(
        self: *PathQueue,
        start_ref: PolyRef,
        end_ref: PolyRef,
        start_pos: [3]f32,
        end_pos: [3]f32,
        filter: *const QueryFilter,
    ) PathQueueRef {
        // Find a free entry.
        const q = for (&self.queue) |*entry| {
            if (entry.ref == path_queue_invalid) break entry;
        } else return path_queue_invalid;

        const ref = self.next_handle;
        self.next_handle +%= 1;
        if (self.next_handle == path_queue_invalid) self.next_handle +%= 1;

        q.ref = ref;
        q.start_pos = start_pos;
        q.start_ref = start_ref;
        q.end_pos = end_pos;
        q.end_ref = end_ref;

        q.status = .{};
        q.path_count = 0;
        q.filter = filter;
        q.keep_alive = 0;

        return ref;
    }

    /// The status of request `ref`: all zero while it waits for its first
    /// `update`, in progress while it is searched, then the search's
    /// outcome with its detail flags. A failure for a handle the queue does
    /// not hold.
    pub fn getRequestStatus(self: *const PathQueue, ref: PathQueueRef) Status {
        for (&self.queue) |*q| {
            if (q.ref == ref) return q.status;
        }
        return .failure;
    }

    /// Copies the path found for request `ref` into `path`, as much as
    /// fits, sets `path_size` to the number of polygons copied, and frees
    /// the request's entry, so `ref` is stale afterwards. Returns success
    /// with the search's detail flags (a partial result keeps its flag), or
    /// a failure for a handle the queue does not hold.
    pub fn getPathResult(
        self: *PathQueue,
        ref: PathQueueRef,
        path: []PolyRef,
        path_size: *usize,
    ) Status {
        for (&self.queue) |*q| {
            if (q.ref == ref) {
                const details = q.status.detail();
                // Free the request for reuse.
                q.ref = path_queue_invalid;
                q.status = .{};
                // Copy the path.
                const n = @min(@as(usize, @intCast(q.path_count)), path.len);
                @memcpy(path[0..n], q.path[0..n]);
                path_size.* = n;
                return details.with(.success);
            }
        }
        return .failure;
    }

    /// The navigation query the requests are searched with. The pointer is
    /// stale after `deinit` or another `init`. A search run on it between
    /// updates discards the queue's search in progress.
    pub fn getNavQuery(self: *PathQueue) *NavMeshQuery {
        return self.nav_query.?;
    }
};
