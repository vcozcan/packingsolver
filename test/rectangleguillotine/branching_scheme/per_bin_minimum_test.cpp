/**
 * Per-bin minimum-distance overrides.
 *
 * These tests exercise the per-bin override mechanism added on top of the
 * (still-supported) global Parameters fields. A bin field set to the sentinel
 * `-1` falls back to the global default; any non-negative value overrides it.
 *
 * The tests cover:
 *   1. setter contract + bounds-check
 *   2. effective_* helper resolution (sentinel vs. set value)
 *   3. propagation through `add_bin_type(const BinType&, ...)` — the P1 path
 *      used by InstanceFlipper, column generation, SVC, and dichotomic search
 *   4. InstanceFlipper preserves per-bin values across the flip (no swap)
 *   5. coexistence with other per-bin features (defects, copies_min)
 *   6. one branching-scheme integration test confirming that the per-bin
 *      breaking distance changes the produced insertions in `update`.
 */

#include "packingsolver/rectangleguillotine/instance_builder.hpp"
#include "rectangleguillotine/branching_scheme.hpp"
#include "rectangleguillotine/instance_flipper.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

using namespace packingsolver;
using namespace packingsolver::rectangleguillotine;

// ---------------------------------------------------------------------------
// 1. Setter wires through to the BinType field and effective_* resolves it.
// ---------------------------------------------------------------------------
TEST(RectangleGuillotineBranchingScheme, PerBinMwlSetterAndEffectiveResolve)
{
    InstanceBuilder builder;
    builder.set_objective(Objective::BinPackingWithLeftovers);
    builder.set_roadef2018();
    builder.set_minimum_waste_length(10);  // global default
    builder.add_item_type(500, 500, -1, 1, false, 0);
    BinTypeId bin_a = builder.add_bin_type(6000, 3210);
    BinTypeId bin_b = builder.add_bin_type(6000, 3210);
    builder.set_bin_minimum_waste_length(bin_b, 50);
    Instance instance = builder.build();

    // Bin A inherits the sentinel.
    EXPECT_EQ(instance.bin_type(bin_a).minimum_waste_length, -1);
    EXPECT_EQ(
            effective_minimum_waste_length(
                    instance.bin_type(bin_a), instance.parameters()),
            10);

    // Bin B carries the per-bin value.
    EXPECT_EQ(instance.bin_type(bin_b).minimum_waste_length, 50);
    EXPECT_EQ(
            effective_minimum_waste_length(
                    instance.bin_type(bin_b), instance.parameters()),
            50);
}

// ---------------------------------------------------------------------------
// 2. Setter bounds-checks: bin_type_id out of range AND value < -1 both throw.
// ---------------------------------------------------------------------------
TEST(RectangleGuillotineBranchingScheme, PerBinMwlSetterRejectsBadInput)
{
    InstanceBuilder builder;
    builder.set_objective(Objective::BinPackingWithLeftovers);
    builder.set_roadef2018();
    builder.add_item_type(500, 500, -1, 1, false, 0);
    BinTypeId bin_id = builder.add_bin_type(6000, 3210);

    // value < -1 is rejected.
    EXPECT_THROW(
            builder.set_bin_minimum_waste_length(bin_id, -5),
            std::invalid_argument);
    EXPECT_THROW(
            builder.set_bin_minimum_distance_1_cuts(bin_id, -2),
            std::invalid_argument);
    EXPECT_THROW(
            builder.set_bin_minimum_distance_2_cuts(bin_id, -3),
            std::invalid_argument);

    // Out-of-range bin_type_id is rejected.
    EXPECT_THROW(
            builder.set_bin_minimum_waste_length(99, 10),
            std::invalid_argument);
    EXPECT_THROW(
            builder.set_bin_minimum_distance_1_cuts(99, 10),
            std::invalid_argument);
    EXPECT_THROW(
            builder.set_bin_minimum_distance_2_cuts(99, 10),
            std::invalid_argument);

    // -1 sentinel is explicitly allowed.
    builder.set_bin_minimum_waste_length(bin_id, -1);
    builder.set_bin_minimum_distance_1_cuts(bin_id, -1);
    builder.set_bin_minimum_distance_2_cuts(bin_id, -1);
}

// ---------------------------------------------------------------------------
// 3. P1 regression guard.
//
// `add_bin_type(const BinType&, ...)` (the copy overload) is used by
// InstanceFlipper, column generation, SVC, and dichotomic search. Without
// the propagation fix in instance_builder.cpp, the per-bin fields would be
// silently zero'd on any sub-instance.
// ---------------------------------------------------------------------------
TEST(RectangleGuillotineBranchingScheme, CopyOverloadPropagatesPerBinFields)
{
    // Build the original instance with per-bin overrides on a single bin type.
    InstanceBuilder original;
    original.set_objective(Objective::BinPackingWithLeftovers);
    original.set_roadef2018();
    original.add_item_type(500, 500, -1, 1, false, 0);
    BinTypeId orig_bin = original.add_bin_type(6000, 3210);
    original.set_bin_minimum_waste_length(orig_bin, 25);
    original.set_bin_minimum_distance_1_cuts(orig_bin, 30);
    original.set_bin_minimum_distance_2_cuts(orig_bin, 35);
    Instance original_instance = original.build();

    // Copy that bin type into a fresh builder via the copy overload.
    InstanceBuilder copied;
    copied.set_objective(Objective::BinPackingWithLeftovers);
    copied.set_roadef2018();
    copied.add_item_type(500, 500, -1, 1, false, 0);
    const BinType& source = original_instance.bin_type(orig_bin);
    copied.add_bin_type(source, source.copies, source.copies_min);
    Instance copied_instance = copied.build();

    // All three per-bin fields must round-trip.
    const BinType& dst = copied_instance.bin_type(0);
    EXPECT_EQ(dst.minimum_waste_length, 25);
    EXPECT_EQ(dst.minimum_distance_1_cuts, 30);
    EXPECT_EQ(dst.minimum_distance_2_cuts, 35);
}

// ---------------------------------------------------------------------------
// 4. InstanceFlipper preserves per-bin values across the flip.
//
// The flipper swaps trims geometrically (left<->bottom, right<->top) but
// the cut distances are stage-hierarchical, NOT directional. Per-bin values
// must travel through unchanged.
// ---------------------------------------------------------------------------
TEST(RectangleGuillotineBranchingScheme, FlipperPreservesPerBinValues)
{
    InstanceBuilder builder;
    builder.set_objective(Objective::BinPackingWithLeftovers);
    builder.set_roadef2018();
    builder.set_first_stage_orientation(CutOrientation::Any);
    builder.add_item_type(500, 500, -1, 1, false, 0);
    BinTypeId bin_id = builder.add_bin_type(6000, 3210);
    builder.set_bin_minimum_waste_length(bin_id, 25);
    builder.set_bin_minimum_distance_1_cuts(bin_id, 30);
    builder.set_bin_minimum_distance_2_cuts(bin_id, 35);
    Instance original = builder.build();

    InstanceFlipper flipper(original);
    const Instance& flipped = flipper.flipped_instance();

    ASSERT_EQ(flipped.number_of_bin_types(), 1);
    const BinType& flipped_bin = flipped.bin_type(0);
    EXPECT_EQ(flipped_bin.minimum_waste_length, 25);
    EXPECT_EQ(flipped_bin.minimum_distance_1_cuts, 30);  // NOT swapped
    EXPECT_EQ(flipped_bin.minimum_distance_2_cuts, 35);  // NOT swapped
}

// ---------------------------------------------------------------------------
// 5. Explicit `-1` sentinel resolves to the global default.
// ---------------------------------------------------------------------------
TEST(RectangleGuillotineBranchingScheme, SentinelEqualsGlobal)
{
    InstanceBuilder builder;
    builder.set_objective(Objective::BinPackingWithLeftovers);
    builder.set_roadef2018();
    builder.set_minimum_waste_length(50);
    builder.set_minimum_distance_1_cuts(60);
    builder.set_minimum_distance_2_cuts(70);
    builder.add_item_type(500, 500, -1, 1, false, 0);
    BinTypeId bin_id = builder.add_bin_type(6000, 3210);
    // Explicit -1 — should resolve to the global default.
    builder.set_bin_minimum_waste_length(bin_id, -1);
    builder.set_bin_minimum_distance_1_cuts(bin_id, -1);
    builder.set_bin_minimum_distance_2_cuts(bin_id, -1);
    Instance instance = builder.build();

    EXPECT_EQ(
            effective_minimum_waste_length(
                    instance.bin_type(bin_id), instance.parameters()),
            50);
    EXPECT_EQ(
            effective_minimum_distance_1_cuts(
                    instance.bin_type(bin_id), instance.parameters()),
            60);
    EXPECT_EQ(
            effective_minimum_distance_2_cuts(
                    instance.bin_type(bin_id), instance.parameters()),
            70);
}

// ---------------------------------------------------------------------------
// 6. Two bins coexist: one inherits the global default, the other overrides.
// ---------------------------------------------------------------------------
TEST(RectangleGuillotineBranchingScheme, MixedGlobalAndPerBinResolveIndependently)
{
    InstanceBuilder builder;
    builder.set_objective(Objective::BinPackingWithLeftovers);
    builder.set_roadef2018();
    builder.set_minimum_waste_length(30);
    builder.add_item_type(500, 500, -1, 1, false, 0);
    BinTypeId bin_a = builder.add_bin_type(6000, 3210);  // inherits global
    BinTypeId bin_b = builder.add_bin_type(6000, 3210);
    builder.set_bin_minimum_waste_length(bin_b, 15);     // overrides
    Instance instance = builder.build();

    EXPECT_EQ(
            effective_minimum_waste_length(
                    instance.bin_type(bin_a), instance.parameters()),
            30);
    EXPECT_EQ(
            effective_minimum_waste_length(
                    instance.bin_type(bin_b), instance.parameters()),
            15);
}

// ---------------------------------------------------------------------------
// 7. Per-bin minimum_distance_1_cuts — same setter / effective_ pattern.
// ---------------------------------------------------------------------------
TEST(RectangleGuillotineBranchingScheme, PerBinMinimumDistance1Cuts)
{
    InstanceBuilder builder;
    builder.set_objective(Objective::BinPackingWithLeftovers);
    builder.set_roadef2018();
    builder.set_minimum_distance_1_cuts(20);
    builder.add_item_type(500, 500, -1, 1, false, 0);
    BinTypeId bin_a = builder.add_bin_type(6000, 3210);
    BinTypeId bin_b = builder.add_bin_type(6000, 3210);
    builder.set_bin_minimum_distance_1_cuts(bin_b, 80);
    Instance instance = builder.build();

    EXPECT_EQ(
            effective_minimum_distance_1_cuts(
                    instance.bin_type(bin_a), instance.parameters()),
            20);
    EXPECT_EQ(
            effective_minimum_distance_1_cuts(
                    instance.bin_type(bin_b), instance.parameters()),
            80);
}

// ---------------------------------------------------------------------------
// 8. Per-bin minimum_distance_2_cuts — same setter / effective_ pattern.
// ---------------------------------------------------------------------------
TEST(RectangleGuillotineBranchingScheme, PerBinMinimumDistance2Cuts)
{
    InstanceBuilder builder;
    builder.set_objective(Objective::BinPackingWithLeftovers);
    builder.set_roadef2018();
    builder.set_minimum_distance_2_cuts(40);
    builder.add_item_type(500, 500, -1, 1, false, 0);
    BinTypeId bin_a = builder.add_bin_type(6000, 3210);
    BinTypeId bin_b = builder.add_bin_type(6000, 3210);
    builder.set_bin_minimum_distance_2_cuts(bin_b, 5);
    Instance instance = builder.build();

    EXPECT_EQ(
            effective_minimum_distance_2_cuts(
                    instance.bin_type(bin_a), instance.parameters()),
            40);
    EXPECT_EQ(
            effective_minimum_distance_2_cuts(
                    instance.bin_type(bin_b), instance.parameters()),
            5);
}

// ---------------------------------------------------------------------------
// 9. Per-bin overrides coexist with defects through the copy overload.
//
// The copy overload reconstructs the bin via add_bin_type+add_trims+defects
// loop, then propagates per-bin fields. This test confirms BOTH paths work.
// ---------------------------------------------------------------------------
TEST(RectangleGuillotineBranchingScheme, PerBinWithDefectsCopyOverload)
{
    InstanceBuilder original;
    original.set_objective(Objective::BinPackingWithLeftovers);
    original.set_roadef2018();
    original.add_item_type(500, 500, -1, 1, false, 0);
    BinTypeId orig_bin = original.add_bin_type(6000, 3210);
    original.add_defect(orig_bin, 100, 100, 50, 50);
    original.add_defect(orig_bin, 200, 200, 30, 30);
    original.set_bin_minimum_waste_length(orig_bin, 25);
    Instance original_instance = original.build();

    InstanceBuilder copied;
    copied.set_objective(Objective::BinPackingWithLeftovers);
    copied.set_roadef2018();
    copied.add_item_type(500, 500, -1, 1, false, 0);
    const BinType& source = original_instance.bin_type(orig_bin);
    copied.add_bin_type(source, source.copies, source.copies_min);
    Instance copied_instance = copied.build();

    const BinType& dst = copied_instance.bin_type(0);
    // Per-bin override propagated.
    EXPECT_EQ(dst.minimum_waste_length, 25);
    // Defects also propagated (existing behaviour, regression-guarded here).
    ASSERT_EQ(dst.defects.size(), source.defects.size());
    EXPECT_EQ(dst.defects[0].pos.x, 100);
    EXPECT_EQ(dst.defects[0].pos.y, 100);
    EXPECT_EQ(dst.defects[1].pos.x, 200);
    EXPECT_EQ(dst.defects[1].pos.y, 200);
}

// ---------------------------------------------------------------------------
// 10. Per-bin overrides coexist with copies_min through the copy overload.
// ---------------------------------------------------------------------------
TEST(RectangleGuillotineBranchingScheme, PerBinWithCopiesMinCopyOverload)
{
    InstanceBuilder original;
    original.set_objective(Objective::BinPackingWithLeftovers);
    original.set_roadef2018();
    original.add_item_type(500, 500, -1, 1, false, 0);
    // copies = 5, copies_min = 2.
    BinTypeId orig_bin = original.add_bin_type(6000, 3210, -1, 5, 2);
    original.set_bin_minimum_distance_1_cuts(orig_bin, 45);
    Instance original_instance = original.build();

    InstanceBuilder copied;
    copied.set_objective(Objective::BinPackingWithLeftovers);
    copied.set_roadef2018();
    copied.add_item_type(500, 500, -1, 1, false, 0);
    const BinType& source = original_instance.bin_type(orig_bin);
    copied.add_bin_type(source, source.copies, source.copies_min);
    Instance copied_instance = copied.build();

    const BinType& dst = copied_instance.bin_type(0);
    EXPECT_EQ(dst.copies, 5);
    EXPECT_EQ(dst.copies_min, 2);
    EXPECT_EQ(dst.minimum_distance_1_cuts, 45);
}

// ---------------------------------------------------------------------------
// 11. Branching integration — per-bin minimum_waste_length actually changes
// the produced insertions in `update`, exactly the same way the global value
// would.
//
// Two scenarios, both with the global default mwl = 0:
//   A. Bin has no per-bin override (effective mwl = 0). Mirrors CutThickness2:
//      the partial-cut variant of item 1 (5990 wide, leaving a 10-unit
//      residual) IS produced.
//   B. Bin has per-bin override mwl = 10 (effective mwl = 10). Mirrors
//      CutThickness3: the partial-cut variant is forbidden; no further
//      insertion is produced from `node_1`.
//
// Everything else — items, bin size, cut_thickness, global parameters — is
// identical between A and B. The differential proves that flipping ONLY the
// per-bin field changes branching the same way changing the global would.
// ---------------------------------------------------------------------------
TEST(RectangleGuillotineBranchingScheme, PerBinMwlAffectsBranchingResidualPadding)
{
    // ----- Scenario A: no per-bin override; effective mwl = 0. -----
    InstanceBuilder builder_a;
    builder_a.set_objective(Objective::BinPackingWithLeftovers);
    builder_a.set_cut_thickness(20);
    builder_a.add_item_type(3000, 500, -1, 1, false, 0);
    builder_a.add_item_type(2970, 3210, -1, 1, false, 0);
    builder_a.add_bin_type(6000, 3210);
    Instance instance_a = builder_a.build();

    BranchingScheme bs_a(instance_a);
    auto root_a = bs_a.root();

    BranchingScheme::Insertion i0_a
            = {0, -1, -1, 3000, 500, 3000, 6000, 3210, 1, 1};
    std::vector<BranchingScheme::Insertion> is0_a =
            bs_a.insertions(bs_a.children(root_a));
    EXPECT_NE(std::find(is0_a.begin(), is0_a.end(), i0_a), is0_a.end());
    auto node_a = bs_a.child(root_a, i0_a);

    // Partial-cut variant (5990 wide) is the only allowed follow-up — same
    // as CutThickness2.
    std::vector<BranchingScheme::Insertion> is_a {
        {1, -1, 0, 5990, 3210, 5990, 6000, 3210, 1, 1},
    };
    EXPECT_EQ(bs_a.insertions(bs_a.children(node_a)), is_a);

    // ----- Scenario B: identical setup, plus per-bin override mwl = 10. -----
    InstanceBuilder builder_b;
    builder_b.set_objective(Objective::BinPackingWithLeftovers);
    builder_b.set_cut_thickness(20);
    builder_b.add_item_type(3000, 500, -1, 1, false, 0);
    builder_b.add_item_type(2970, 3210, -1, 1, false, 0);
    BinTypeId bin_b = builder_b.add_bin_type(6000, 3210);
    builder_b.set_bin_minimum_waste_length(bin_b, 10);
    Instance instance_b = builder_b.build();

    BranchingScheme bs_b(instance_b);
    auto root_b = bs_b.root();

    // With effective mwl = 10, `update` skips the `min_waste <= 1` branch that
    // forces z1/z2 to 1, so the expected first insertion has z1 = z2 = 0 —
    // same as CutThickness3.
    BranchingScheme::Insertion i0_b
            = {0, -1, -1, 3000, 500, 3000, 6000, 3210, 0, 0};
    std::vector<BranchingScheme::Insertion> is0_b =
            bs_b.insertions(bs_b.children(root_b));
    EXPECT_NE(std::find(is0_b.begin(), is0_b.end(), i0_b), is0_b.end());
    auto node_b = bs_b.child(root_b, i0_b);

    // Partial-cut variant is now forbidden; no follow-up insertion remains —
    // same as CutThickness3.
    std::vector<BranchingScheme::Insertion> is_b {};
    EXPECT_EQ(bs_b.insertions(bs_b.children(node_b)), is_b);
}
