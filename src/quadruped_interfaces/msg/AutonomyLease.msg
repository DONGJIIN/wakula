# Session-scoped ownership heartbeat for the autonomous Nav2 velocity branch.
#
# ``std_msgs/Bool`` cannot distinguish a delayed release from a previous mission
# process from the current owner.  A new mission creates one unpredictable session ID
# and increases ``sequence`` for every heartbeat and final release.  Consumers must
# accept a release only for the currently active session and only when its sequence is
# strictly newer than every message already seen for that session.

# Opaque, non-empty owner identity.  The repository producer uses a lowercase UUID4
# hexadecimal string; consumers must not infer robot or task semantics from it.
string session_id

# Starts at 1 and increases monotonically inside one session.  Zero, a repeated value,
# or a value lower than the session high-water mark is invalid and must not refresh or
# release ownership.
uint64 sequence

# true acquires/refreshes ownership; false cleanly releases only the matching active
# session.  A different session's false must never release the current owner.
bool active

# Atomic permission for the current session's Nav2 velocity branch.  Acquisition is
# always fail-closed: a consumer must force this field false on the first active sample
# of a previously unowned session, even if that sample says true.  Only a later,
# strictly newer active sample from the same session may enable motion.  The producer
# sends false before cancellation, handoff, shutdown or any uncertain Action boundary.
# This field must be false when active is false.
bool motion_allowed
