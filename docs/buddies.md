# Buddies Feature — rectangleguillotine

## Overview

Buddies are a **same-plate co-location** constraint for glass cutting scenarios
where a group of pieces must all be cut from one single plate (bin) — never
split across plates. Every item sharing a `BUDDY_ID` (across all rows and all
copies) must land on **one single bin, and that bin only**.

Buddies are a third grouping axis, independent of stacks and sets, and
semantically the opposite of both:

- **stacks** force cut *order/precedence*; members scatter across bins.
- **sets** force *non-interleaving* of fixed-size sub-groups; a sub-group may
  legally *straddle* bin boundaries (`solution.cpp`).
- **buddies** force *co-location*; a group is a hard, all-or-nothing unit that
  occupies exactly one plate.

The guarantee is **hard**: if a group cannot fit on a single plate, the job is
infeasible (no fallback). A group is placed in full and co-located, or not at
all.

## CSV Schema

Buddy items are specified with one column in the items CSV:

| Column     | Type | Description |
|------------|------|-------------|
| `BUDDY_ID` | int  | Identifies which buddy group this item type belongs to. Use the same value for all rows in the same group. Arbitrary non-negative integers (sparse values like 50123 are fine — they are remapped to dense indices internally). Use `-1` or leave blank for non-buddy items. |

There is no per-row size column (unlike `SET_SIZE`): a group is simply *all*
copies of *all* rows sharing the `BUDDY_ID`.

### Example

```csv
WIDTH,HEIGHT,COPIES,BUDDY_ID
1000,500,2,0
600,400,2,0
800,300,3,-1
```

This defines:
- Two item types in buddy group 0 (4 pieces total: 2 + 2) that must all be cut
  from the same single plate.
- One non-buddy item type (800x300) with 3 copies, free to go anywhere.

### Group total must be at least 2

A buddy group must contain at least 2 pieces in total (summed across all rows
and copies). A single-piece group is meaningless and is rejected at build time.

### Mutual exclusion with SET_ID and STACK_ID

`BUDDY_ID` **cannot coexist with `SET_ID` or an explicit `STACK_ID` on the same
item**. Within the same instance, different items may freely use different
features — some items can have `BUDDY_ID`, others `SET_ID`, others `STACK_ID`,
others none. This per-item constraint is enforced at build time with an
`std::invalid_argument` exception.

### Output schema

When `write()` exports a buddy-enabled instance, the CSV header appends
`BUDDY_ID` (independently of `SET_ID`/`SET_SIZE`, so all four combinations of
the two features are produced). Non-buddy items get `-1`.

## Constraint Semantics

### Same-plate co-location

All pieces of a buddy group must be cut from one single plate. The search is a
forward-only chain — items go into the current last bin, a bin is left *only*
via the `df < 0` transition that opens a new bin, and a closed bin never
reopens. The constraint is therefore a single rule:

> **While any buddy group is "open" (partially placed: `0 < placed < total`),
> the search must not open a new bin.**

Once a group's first piece is placed, the group is open, and the only legal
moves are within the current bin until every piece of the group is placed. If
the remaining pieces cannot fit, that branch dead-ends and the search
backtracks to try another arrangement. If no single-plate arrangement exists,
the group is never fully placed (no full solution under the hard guarantee).

### All-or-nothing

A group is either fully placed (all copies co-located on one plate) or entirely
absent. Under `Objective::Knapsack`, a group may be dropped (0 copies placed),
but it is never retained half-placed.

### No ordering, no adjacency

Buddies impose no cut order and no adjacency — only co-location. Non-buddy items
and items of other groups interleave freely; another group's pieces may share
the same plate.

## Internal Implementation

### Data model

- `BuddyId` is `int32_t` (defined in `instance.hpp`).
- `ItemType` has a `buddy_id` field (default: -1).
- `Instance` has per-stack/per-group metadata: `buddy_id_per_stack_[]`,
  `buddy_stacks_[]` (dense group index → list of stack IDs), and
  `buddy_total_[]` (dense group index → total item copies in the group).

### Dense remapping

User-provided `BUDDY_ID` values (which can be sparse, e.g., 50123) are remapped
to dense internal indices (0, 1, 2, ...) during `build()`. The original
`BUDDY_ID` is preserved on `ItemType::buddy_id` for output traceability. The
dense indices are used only in the per-stack/per-group arrays.

### Stack construction

Each buddy item type becomes a singleton stack (one stack per item type), the
same way set items do: `BUDDY_ID` items have `stack_id = -1`, so `build()`
auto-assigns them sequential singleton stacks.

### Build-time area precheck (infeasibility necessary condition)

Because the guarantee is hard, a group that cannot fit on any single plate makes
the whole job infeasible — there is no fallback. `build()` performs a necessary
condition: for each group, if the sum of `area × copies` exceeds the largest
usable (trimmed) bin area (`BinType::area()`, all trims subtracted), it throws an
`std::invalid_argument` naming the group. Area is rotation-invariant, so this is
orientation-independent.

The bound subtracts all four trims regardless of trim type, because piece
placement is confined to the trimmed area for every trim type (verified
empirically): soft trims only relax `min_waste` near the trim and let the
trailing leftover overhang to the physical edge — they do not move pieces into
the trim band. So `BinType::area()` is the correct, tight bound, not an
over-rejection.

This is *necessary, not sufficient*: a group can pass the area check yet still
be unfittable for guillotine/aspect/defect reasons. That case surfaces as a
solver no-solution (the group is never fully placed) rather than a clean build
error. A single-bin trial-pack is a future tightening.

### Symmetry breaking (stack_pred_)

The `stack_pred_` array provides symmetry breaking by linking duplicate stacks.
For buddy instances, a two-pass post-processing step runs after the standard
construction (and composes with the analogous set pass):

1. **Pass 1 — break:** Break links whose two stacks are in different buddy
   groups (including a buddy stack linked to a free or set stack). Two
   identical-geometry stacks in different groups are *not* interchangeable —
   each carries its own co-location constraint.
2. **Pass 2 — relink:** Relink broken entries to a valid predecessor within the
   same buddy group (skipping `buddy_id == -1`, so free/set stacks are never
   relinked by this pass).

The set pass and the buddy pass compose: each relinks only its own group type
(skipping the sentinel `-1`) and treats the other type as `-1`.

### Node and NodeHasher

No changes to `Node` or `NodeHasher` were needed. Because buddy groups are
single-occurrence (one name = one plate, no repeating sub-groups), the
"open" state is fully derivable from the existing `pos_stack[s]` values
(`buddies_open(node)`). Same `pos_stack` ⇒ same buddy state.

### Internal subinstance copies (SVC, SSK, DS, CG, InstanceFlipper)

The `add_item_type(const ItemType&, Profit, ItemPos)` overload propagates
`buddy_id` and keeps the materialized `stack_id` unchanged, exactly as it does
for `set_id`. Copied buddy items legitimately carry both `buddy_id` and a
materialized `stack_id`; they are exempted from the user-input mutual-exclusion
check via the builder-private `internal_copy_item_type_ids_` registry, with the
same singleton-stack shape check that sets use. (In v1 the non-TS algorithms are
gated off for buddies. The metadata propagation is wired, but enabling them is
**not** purely additive — see Known Limitations.)

### Solution-level companion checker

`Solution::update_indicators()` records, per dense group, the set of bins its
items appear in, and flags `buddies_feasible() == false` /
`feasible() == false` if a group spans more than one bin or lands in a
replicated (`copies > 1`) pattern bin (which would scatter its pieces across
`copies` physical plates). The end-state partial-group check (a group must be
fully placed or entirely absent — never `0 < placed < total`) is folded into the
`buddies_feasible()` accessor, which therefore subsumes completeness (there is
no separate `buddies_complete()` on `Solution`).

This is independent defense-in-depth: it re-validates `SolutionBuilder` output
and any future algorithm, not just the forward search.
`BranchingScheme::to_solution()` throws if its own full reconstruction fails the
check (last-bin-only `json_export` reconstructions are exempt, like sets).

### Branching retention rule

`BranchingScheme::better()` only retains a node when no buddy group is open
(`buddies_complete(node)`), under `Objective::Default` and `Objective::Knapsack`.
The fullness objectives (BinPacking, BinPackingWithLeftovers,
VariableSizedBinPacking, OpenDimension*) gate on `leaf()`, and a leaf has every
item placed, so every group is necessarily complete and — thanks to the new-bin
guard — co-located.

## Algorithm Restrictions

Buddies are **tree-search-only** in v1.

| Objective | Tree Search | SSK / SVC / DS | CG / CG2 |
|-----------|-------------|----------------|----------|
| BinPacking / BinPackingWithLeftovers | yes (forced) | no | no |
| VariableSizedBinPacking, single bin type | yes (forced) | no | no |
| VariableSizedBinPacking, **multiple bin types** | **rejected** | no | no |
| Knapsack / OpenDimension / Default | yes (forced) | no | no |

**Behavior:**
- The same-plate guarantee is enforced by the branching scheme's new-bin guard,
  which exists only in tree search. SSK/SVC/DS and column generation have no
  equivalent and throw `std::invalid_argument`.
- Multi-bin-type variable-sized bin packing is **rejected**: tree search is
  unavailable there (the forward chain cannot select among bin types), so the
  guarantee cannot be honoured. Use a single bin type.
- The gate reads the raw requested flags (like the sets gate), so an explicit
  incompatible request is rejected even when an upstream normalization already
  cleared the local flag.
- `buddies_complete()` is pre-wired into `better()`, but SSK/SVC/DS parity is
  **not** purely additive: those algorithms emit a single pattern replicated
  `copies` times, which the solution certificate's replicated-host rule
  (`bin.copies > 1` ⇒ infeasible) rejects even for a legitimately co-located
  group. Enabling them needs a co-location-aware replicated-pattern check, not
  just flipping the gate.

### Performance

`buddies_open()` iterates each group's stacks per `children()` expansion —
O(total buddy stacks) per expansion. For typical glass-cutting groups this is
negligible.

## Validation Rules

All validation runs in `InstanceBuilder::build()`. Invalid input throws
`std::invalid_argument`.

| Rule | Error message (abridged) |
|------|--------------------------|
| Same item has both `BUDDY_ID` and `SET_ID` | "item type N has both SET_ID and BUDDY_ID" |
| Same item has both `BUDDY_ID` and explicit `STACK_ID` | "item type N has both BUDDY_ID and explicit STACK_ID" |
| `BUDDY_ID` < 0 (other than -1) | "item type N has negative BUDDY_ID (X)" |
| Group total < 2 | "buddy group G has only K item(s); a buddy group must contain at least 2 items" |
| Group cannot fit any single plate (by area, trim-aware) | "buddy group G has total item area A which exceeds the largest usable bin area (B)" |

The multi-bin-type-VBPP rejection lives in `optimize()`, not `build()`.

## Known Limitations

1. **Tree-search-only; no multi-bin-type VBPP:** see Algorithm Restrictions.
   `buddies_complete()` is pre-wired into `better()`, but non-TS parity is **not**
   purely additive — the replicated-host rule (`bin.copies > 1` ⇒ infeasible)
   rejects the single-pattern replicated output SSK/SVC/DS produce, so enabling
   them requires a co-location-aware replicated-pattern check.

2. **Area precheck is necessary, not sufficient:** a group can pass the area
   check yet be unfittable for guillotine/aspect/defect reasons, surfacing as a
   solver no-solution rather than a clean build error. A single-bin trial-pack
   is the future tightening.

3. **`set_last_item_type_buddy()` coupling:** Must be called immediately after
   `add_item_type()`. All validation is deferred to `build()`.

## Test Coverage

Automated tests in `test/rectangleguillotine/buddies_test.cpp`:

- **Instance building:** basic group, single-row multi-copy group, mixed
  buddy/non-buddy, multiple independent groups, sparse BUDDY_IDs (dense
  remap + original preserved), non-buddy regression.
- **Validation:** buddy+set and buddy+stack mutual exclusion, negative
  BUDDY_ID, singleton group rejected, group-too-big-by-area (with and without
  trims), mutual exclusion still rejected after copy support, builder reuse.
- **Internal copy / flipper:** mixed-composition flipper round trip preserving
  stack structure and buddy metadata.
- **Branching scheme:** cross-group `stack_pred_` breaking across free / set /
  multiple buddy groups of identical geometry.
- **Enforcement (optimize):** single group co-locates, multi-row + multi-copy
  groups co-locate with free items, a group that cannot co-locate (passes the
  area precheck) is dropped entirely under Knapsack, tree search solves a buddy
  instance to a co-located full placement.
- **Solution checker (SolutionBuilder):** co-located complete (feasible), split
  across bins (infeasible), partial group (infeasible), replicated host
  (infeasible), copies>1 item co-located on one plate (feasible), untouched
  group (feasible), sparse-id dense indexing.
- **Algorithm selection:** multi-bin VBPP rejected, explicit CG rejected,
  explicit SVC rejected.

## Files Modified

| File | Changes |
|------|---------|
| `instance.hpp` | `BuddyId` typedef, `buddy_id` on `ItemType`, buddy metadata + accessors on `Instance` |
| `instance_builder.hpp` | `set_last_item_type_buddy()` declaration |
| `instance_builder.cpp` | CSV parsing, validation, dense remap, `buddy_total_`, area precheck, buddy-propagating internal copy overload |
| `solution.hpp` / `solution.cpp` | `buddies_feasible()` companion checker in `update_indicators()` |
| `branching_scheme.hpp` | `buddy_placed`/`buddies_open`/`buddies_complete` helpers |
| `branching_scheme.cpp` | Two-pass buddy `stack_pred_` fix, new-bin guard, `better()` retention rule, `to_solution()` buddy tripwire |
| `optimize.cpp` | Buddy gate (tree-search-only; CG/SSK/SVC/DS rejected; multi-bin-type VBPP rejected) |
| `instance.cpp` | Buddy-aware `write()` (independent of sets), `operator<<` |
| `test/rectangleguillotine/buddies_test.cpp` | Buddy test suite |
