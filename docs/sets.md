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

`SET_ID` and `STACK_ID` **cannot coexist** in the same instance. If any item
has a `SET_ID`, no item may have a `STACK_ID`, and vice versa. This is enforced
at build time with an `std::invalid_argument` exception.

This is a product-level restriction (not a technical limitation) — the
branching logic could handle mixed modes, but the use case does not require it.

### Output schema

When `write()` exports a set-enabled instance, the CSV header uses
`SET_ID,SET_SIZE` instead of `STACK_ID`. Non-set items in a set instance get
`-1,-1` for the set columns. The `STACK_ID` column is omitted entirely because
stack assignments are auto-computed by `build()` and carry no user-provided
semantic identity.

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

### InstanceFlipper

The `InstanceFlipper` (used for trying both horizontal and vertical first-cut
orientations) is safe for sets. The branching scheme queries `instance_`
(the original, unflipped instance) for all set metadata. Stack indices, sizes,
and set metadata are invariant under flipping.

## Algorithm Restrictions

### Tree search only

Set instances **only support tree search**. The following algorithms are
incompatible and cannot be used:

- Column Generation (CG)
- Column Generation 2 (CG2)
- Sequential Value Correction (SVC)
- Sequential Single Knapsack (SSK)
- Dichotomic Search (DS)

**Why:** These algorithms create subinstances via `add_item_type(const
ItemType&, profit, copies)`, which does not propagate `set_id` or `set_size`.
Additionally, the cross-bin active-row state is lost between subproblems.

**Behavior:**
- If the user **explicitly requests** an incompatible algorithm (e.g.,
  `--use-sequential-value-correction`), the solver throws
  `std::invalid_argument`.
- If the auto-selection logic would choose an incompatible algorithm, it is
  silently overridden to tree search.

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
| `SET_ID` and `STACK_ID` both present | "SET_ID and STACK_ID are mutually exclusive" |
| `SET_SIZE` without `SET_ID` (orphan) | "item type N has SET_SIZE but no SET_ID" |
| `SET_ID` < 0 (other than -1) | "item type N has negative SET_ID (X)" |
| `SET_ID` present but `SET_SIZE` <= 0 | "item type N has SET_ID but missing or invalid SET_SIZE" |
| `COPIES` not divisible by `SET_SIZE` | "item type N copies (C) not divisible by SET_SIZE (S)" |

The orphan SET_SIZE check runs unconditionally (outside the `has_any_set` gate)
to catch cases where no item has a valid SET_ID but some have SET_SIZE set.

## Known Limitations

1. **No mixed SET_ID + STACK_ID:** Cannot combine sets with explicit stacks in
   the same instance. This is a design choice, not a technical limitation.

2. **Tree search only:** Subproblem-based algorithms (SVC, CG, SSK, DS) are
   incompatible. This limits optimization power for large instances where these
   algorithms would otherwise outperform tree search.

3. **Single-bin limitation with high copy counts:** When using
   `BinPackingWithLeftovers` objective with a single bin and very high copy
   counts (e.g., 120 copies), the `NotAnytimeSequential` mode may fail to
   place any items. Use `Anytime` mode or multiple bins for such cases.

4. **`add_item_type(const ItemType&, profit, copies)` drops set metadata:**
   This overload (used by internal algorithms) does not propagate `set_id` or
   `set_size`. This is intentional and guarded by the tree-search-only
   restriction, but future algorithm development must be aware of this.

5. **`set_last_item_type_set()` coupling:** Must be called immediately after
   `add_item_type()`. There is no validation at the call site — all validation
   is deferred to `build()`.

6. **`set_size_of_stack()` accessor:** Returns -1 for non-set stacks. Callers
   must check `set_id_of_stack(s) != -1` before using the return value.

## Test Coverage

22 automated tests in `test/rectangleguillotine/sets_test.cpp`:

- **Instance building (5):** basic set, mixed set/non-set, multiple sets,
  sparse SET_IDs, non-set regression
- **Validation (7):** mutual exclusion, copies divisibility, SET_SIZE without
  SET_ID, orphan SET_SIZE, negative SET_ID, missing SET_SIZE, SET_SIZE=0
- **CSV parsing (3):** basic, mixed, multiple sets
- **Branching scheme (2):** cross-set stack_pred_ breaking, different-SET_SIZE
  stack_pred_ breaking
- **Solve (5):** tree search forcing, incompatible algorithm rejection,
  single-row set, mixed set/non-set solve, multiple sets solve

Test data in `data/rectangleguillotine/tests/sets_*/`.

## Files Modified

| File | Changes |
|------|---------|
| `instance.hpp` | `SetId` typedef, `set_id`/`set_size` on `ItemType`, set metadata + accessors on `Instance` |
| `instance_builder.hpp` | `set_last_item_type_set()` declaration |
| `instance_builder.cpp` | CSV parsing, validation, dense index remapping |
| `branching_scheme.cpp` | Two-pass `stack_pred_` fix, active-row filter, Roadef2018 guard |
| `optimize.cpp` | Tree-search-only enforcement with explicit error on incompatible flags |
| `instance.cpp` | Set-aware `write()`, conditional `format()`, `operator<<` |
