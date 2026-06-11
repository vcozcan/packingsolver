# Sets Feature — rectangleguillotine

## Overview

Sets are a relaxed variant of stacks designed for glass cutting scenarios where
pieces of the same dimension come in pairs, triplets, or other fixed-size
sub-groups (e.g., laminate glass or insulated glass units). Within a set, items
are grouped into sub-groups of `SET_SIZE` copies, and the solver enforces that
only one "row" (item type) may have an in-progress sub-group at a time.

Unlike stacks, sets impose no ordering between different rows — rows can be
interleaved freely as long as each row's sub-groups are completed before
switching to another row.

## CSV Schema

Items in a set are specified with two columns in the items CSV:

| Column     | Type  | Description |
|------------|-------|-------------|
| `SET_ID`   | int   | Identifies which set this item type belongs to. Use the same value for all rows in the same set. Arbitrary non-negative integers (sparse values like 50123 are fine — they are remapped to dense indices internally). Use `-1` or leave blank for non-set items. |
| `SET_SIZE` | int   | Number of copies forming one sub-group within the set. Must be a positive integer. `COPIES` must be evenly divisible by `SET_SIZE`. |

### Example

```csv
WIDTH,HEIGHT,COPIES,SET_ID,SET_SIZE
1000,500,4,0,2
600,400,4,0,2
800,300,3,-1,-1
```

This defines:
- Two item types in set 0, each with 4 copies grouped in pairs (SET_SIZE=2)
- One non-set item type (800x300) with 3 copies, free to interleave anywhere

### Mutual exclusion with STACK_ID

`SET_ID` and `STACK_ID` **cannot coexist on the same item**. Within the same
instance, different items may freely use different features — some items can
have `SET_ID`, others can have `STACK_ID`, and others can have neither.
This per-item constraint is enforced at build time with an
`std::invalid_argument` exception.

### Output schema

When `write()` exports a set-enabled instance, the CSV header includes
`STACK_ID,SET_ID,SET_SIZE`. All items get their `stack_id` (explicit or
auto-assigned by `build()`). Non-set items get `-1,-1` for the set columns.

Round-trip safety: re-reading a `write()` output produces a behaviorally
identical instance. Stack indices are deterministically reassigned in item-type
order by `build()`.

## Constraint Semantics

### Active-row constraint

Within a set, **at most one row may have an in-progress sub-group at any time**.

A sub-group is "in progress" when the number of copies placed from that row is
not a multiple of its `SET_SIZE`. For example, with SET_SIZE=2:
- 0 copies placed: not active (no sub-group started)
- 1 copy placed: **active** (mid-pair)
- 2 copies placed: not active (pair complete)
- 3 copies placed: **active** (mid-pair)

If row A in a set is active, the solver blocks all other rows in the same set
until row A's current sub-group is completed.

### No ordering between rows

Unlike stacks, sets impose no precedence between rows. Row B does not need to
wait for row A to finish all its copies — only for the current sub-group to
complete.

### Free interleaving with non-set items

Non-set items can be placed at any time, regardless of set state. They are
completely independent of the active-row constraint.

### Two-item insertion (Roadef2018 only)

The Roadef2018 cut type supports placing two items in a single insertion. When
both items come from different rows of the same set, an additional check ensures
the combined insertion does not violate the active-row constraint. Other cut
types have no two-item insertion path, so sets are implicitly safe there.

## Internal Implementation

### Data model

- `SetId` is `int32_t` (defined in `instance.hpp`)
- `ItemType` has `set_id` and `set_size` fields (defaults: -1)
- `Instance` has per-stack metadata arrays: `set_id_per_stack_[]`,
  `set_size_per_stack_[]`, and `set_stacks_[]` (mapping dense set index to
  list of stack IDs)

### Dense remapping

User-provided `SET_ID` values (which can be sparse, e.g., 50123) are remapped
to dense internal indices (0, 1, 2, ...) during `build()`. The original
`SET_ID` is preserved on `ItemType::set_id` for output traceability. The dense
indices are used only in `set_id_per_stack_[]` and `set_stacks_[]`.

### Stack construction

Each item type in a set becomes a singleton stack (one stack per item type).
This happens naturally because `SET_ID` items have `stack_id = -1` and `build()`
auto-assigns them sequential singleton stacks. The set constraint then operates
over these stacks via `pos_stack[s] % SET_SIZE`.

### Symmetry breaking (stack_pred_)

The `stack_pred_` array provides symmetry breaking by linking duplicate stacks.
For set instances, a two-pass post-processing step runs after the standard
`stack_pred_` construction:

1. **Pass 1 — break:** Break links where the predecessor is in a different set
   or has a different `SET_SIZE`.
2. **Pass 2 — relink:** Relink broken entries to a valid predecessor within the
   same set, matched by the `equals()` function.

### Node and NodeHasher

No changes to `Node` or `NodeHasher` were needed. Set state is fully derivable
from the existing `pos_stack[s]` values using `pos_stack[s] % SET_SIZE`.

### Internal subinstance copies (SVC, SSK, DS, CG, InstanceFlipper)

The `add_item_type(const ItemType&, Profit, ItemPos)` overload — used by the
subproblem algorithms and the `InstanceFlipper` — propagates `set_id` /
`set_size` and keeps the materialized `stack_id` unchanged, so the copied
instance has an **identical stack structure** to the source. This matters
because the branching scheme reads set metadata from `instance_` (the
original) while `pos_stack` is indexed by the possibly-flipped instance —
stack-index invariance between the two is load-bearing.

Copied set items legitimately carry both `set_id` and a (materialized)
`stack_id`; they are exempted from the user-input mutual-exclusion check via a
builder-private registry (`internal_copy_item_type_ids_`). User input paths
still throw.

### Solution-level companion checker

`Solution::update_indicators()` re-validates the sets invariant on every
emitted bin (mirroring the existing stacks check): bins are replayed
`copies` times sequentially (physical cut order), per-set run state persists
across bins (sub-groups may straddle), and a violation sets
`sets_feasible() == false` / `feasible() == false`. Trailing-incomplete
sub-groups do **not** flip feasibility (bins arrive incrementally); strict
end-state completeness is exposed via `Solution::sets_complete()`.
`BranchingScheme::to_solution()` throws if its own solution fails this check.

### Knapsack acceptance rule

Under `Objective::Knapsack` (the inner objective of SSK/SVC patterns),
`BranchingScheme::better()` only retains nodes whose every set row is
sub-group-complete (`pos_stack[s] % set_size == 0`). Queue admission goes
through `leaf()`/`bound()`, not `better()`, so mid-sub-group nodes remain
explorable — they just cannot be returned as a result. This keeps single-bin
patterns from cutting a sub-group in half and preserves the divisibility of
remaining copies when SVC rebuilds its subinstance.

## Algorithm Restrictions

Support is **per objective**:

| Objective | Tree Search | SSK | SVC | DS | CG / CG2 |
|-----------|-------------|-----|-----|----|----------|
| BinPacking / BinPackingWithLeftovers | yes | yes | yes | n/a (zeroed upstream for all instances) | no |
| VariableSizedBinPacking | yes (single bin type) | yes | yes | yes | no |
| Knapsack / other | yes (forced) | no | no | no | no |

**Behavior:**
- Explicit `--use-column-generation` / `--use-column-generation-2` on a set
  instance throws `std::invalid_argument` under every objective (the pricing
  loop has no per-pattern completeness rule; needs a separate LDS audit).
- Under Knapsack and other out-of-scope objectives, explicit SSK/SVC/DS
  throws and auto-selection is overridden to tree search.
- Under the fullness objectives, auto-selection simply excludes CG/CG2 from
  the pool; SSK/SVC/DS run as selected.
- The gate reads the raw requested flags, so an explicit incompatible request
  is rejected even when an upstream normalization (e.g. the single-bin
  branch) already cleared the local flag.

**Structural note (accepted):** a sub-group that cannot complete within one
bin pattern can never be packed by SSK/SVC (tree search can straddle bins).
This yields a partial solution, never a violation; keep tree search in the
pool as the floor.

### Performance

The active-row check iterates all stacks in the set for every candidate in
`children()`. This is O(stacks_in_set) per candidate per expansion. For
typical glass-cutting sets (2-6 rows per set), this is negligible. For
unusually large sets, performance may degrade.

## Validation Rules

All validation runs in `InstanceBuilder::build()`. Invalid input throws
`std::invalid_argument`.

| Rule | Error message |
|------|---------------|
| Same item has both `SET_ID` and `STACK_ID` | "item type N has both SET_ID and explicit STACK_ID" |
| `SET_SIZE` without `SET_ID` (orphan) | "item type N has SET_SIZE but no SET_ID" |
| `SET_ID` < 0 (other than -1) | "item type N has negative SET_ID (X)" |
| `SET_ID` present but `SET_SIZE` <= 0 | "item type N has SET_ID but missing or invalid SET_SIZE" |
| `COPIES` not divisible by `SET_SIZE` | "item type N copies (C) not divisible by SET_SIZE (S)" |

The orphan SET_SIZE check runs unconditionally (outside the `has_any_set` gate)
to catch cases where no item has a valid SET_ID but some have SET_SIZE set.

## Known Limitations

1. **Column generation unsupported; Knapsack objective is tree-search-only:**
   see Algorithm Restrictions above. SSK/SVC additionally cannot pack a
   sub-group that does not fit within a single bin pattern (tree search can
   straddle bins) — partial solution, never a violation.

2. **Single-bin limitation with high copy counts:** When using
   `BinPackingWithLeftovers` objective with a single bin and very high copy
   counts (e.g., 120 copies), the `NotAnytimeSequential` mode may fail to
   place any items. Use `Anytime` mode or multiple bins for such cases.

3. **`set_last_item_type_set()` coupling:** Must be called immediately after
   `add_item_type()`. There is no validation at the call site — all validation
   is deferred to `build()`.

4. **`set_size_of_stack()` accessor:** Returns -1 for non-set stacks. Callers
   must check `set_id_of_stack(s) != -1` before using the return value.
   This includes the Solution-level checker, which derives dense set ids via
   `set_id_of_stack(stack_id)` — never index per-set state by the sparse
   `ItemType::set_id`.

## Test Coverage

44 automated tests in `test/rectangleguillotine/sets_test.cpp`:

- **Instance building (6):** basic set, mixed set/non-set, multiple sets,
  sparse SET_IDs, non-set regression, mixed set + explicit stack
- **Validation (7):** per-item mutual exclusion, copies divisibility, SET_SIZE
  without SET_ID, orphan SET_SIZE, negative SET_ID, missing SET_SIZE, SET_SIZE=0
- **CSV parsing (3):** basic, mixed, multiple sets
- **Internal copy / flipper (2):** mixed-composition flipper round trip with
  exact stack-mapping asserts, mutual exclusion still rejected on user input
- **Branching scheme (3):** cross-set stack_pred_ breaking, different-SET_SIZE
  stack_pred_ breaking, Knapsack sub-group-complete acceptance
- **Solution checker (5):** interleaving detection, cross-bin straddling,
  lenient trailing, sparse-set-id dense indexing, high-copy bin replay
- **Non-TS enablement end-to-end (4):** SSK+BPP, SVC+BPPL (last-bin reopt),
  SVC+VBPP, DS+VBPP — full placement + certificate oracle
  (`sets_oracle.hpp`, a port of the analysis harness checker)
- **Auto-selection (3):** BPP / BPPL / VBPP-multi-bin with no algorithm flags,
  shaped so the pre-gate pool would have included CG
- **Stacks regressions (2):** SSK+BPP and SVC+VBPP on stack instances
- **Gate / solve (9):** tree search forcing, explicit CG / CG2 rejection,
  Knapsack explicit SVC rejection (multi-bin and single-bin), SVC+BPPL
  allowed end-to-end, single-row set, mixed set/non-set solve, multiple sets
  solve

Test data in `data/rectangleguillotine/tests/sets_*/`.

## Files Modified

| File | Changes |
|------|---------|
| `instance.hpp` | `SetId` typedef, `set_id`/`set_size` on `ItemType`, set metadata + accessors on `Instance` |
| `instance_builder.hpp` | `set_last_item_type_set()` declaration, `internal_copy_item_type_ids_` registry |
| `instance_builder.cpp` | CSV parsing, validation, dense index remapping, set-propagating internal copy overload |
| `solution.hpp` / `solution.cpp` | `sets_feasible()` companion checker in `update_indicators()`, `sets_complete()` |
| `branching_scheme.hpp` | `set_stack_list_`, `sets_complete(Node)` helper |
| `branching_scheme.cpp` | Two-pass `stack_pred_` fix, active-row filter, Roadef2018 guard, Knapsack acceptance rule, `to_solution()` sets tripwire |
| `optimize.cpp` | Per-algorithm, per-objective sets gate (CG rejected; SSK/SVC/DS allowed for fullness objectives) |
| `instance.cpp` | Set-aware `write()`, conditional `format()`, `operator<<` |
| `test/rectangleguillotine/sets_oracle.hpp` | Certificate-level sets oracle used by the end-to-end tests |
