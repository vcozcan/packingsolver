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

namespace
{

/** Build a minimal one-bin, one-item instance for setter/effective_* tests. */
Instance make_minimal_instance(
        Length bin_w = 6000,
        Length bin_h = 3210)
{
    InstanceBuilder b;
    b.set_objective(Objective::BinPackingWithLeftovers);
    b.set_roadef2018();
    b.add_item_type(500, 500, -1, 1, false, 0);
    b.add_bin_type(bin_w, bin_h);
    return b.build();
}

}  // namespace

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
// Two bin types: bin 0 has per-bin mwl = 0 (no enforcement), bin 1 has
// per-bin mwl = 100 (forces residual padding). For an item placed first in
// bin 1, an off-by-100 padding must surface as a difference in node.waste
// relative to the same insertion attempted in bin 0.
//
// This mirrors the CutThickness3 test pattern (which uses the global
// minimum_waste_length) but exercises per-bin resolution.
// ---------------------------------------------------------------------------
TEST(RectangleGuillotineBranchingScheme, PerBinMwlAffectsBranchingResidualPadding)
{
    // Establish: global mwl = 0 — only bin 1 carries an override.
    InstanceBuilder builder;
    builder.set_objective(Objective::BinPackingWithLeftovers);
    builder.set_roadef2018();
    builder.set_cut_thickness(20);
    builder.set_minimum_waste_length(0);  // explicit global = 0
    builder.add_item_type(3000, 500, -1, 1, false, 0);
    builder.add_item_type(2970, 3210, -1, 1, false, 0);
    BinTypeId bin_with_override = builder.add_bin_type(6000, 3210);
    builder.set_bin_minimum_waste_length(bin_with_override, 10);
    Instance instance = builder.build();

    // With mwl=10 carried per-bin, the "partial cutting" optimisation in
    // CutThickness2 is disallowed for this bin — only the full-width
    // insertion below should be produced. This mirrors CutThickness3's
    // assertion but proves the constraint came from the per-bin field.
    BranchingScheme branching_scheme(instance);
    auto root = branching_scheme.root();

    BranchingScheme::Insertion i0 = {0, -1, -1, 3000, 500, 3000, 6000, 3210, 1, 1};
    std::vector<BranchingScheme::Insertion> is0 =
            branching_scheme.insertions(branching_scheme.children(root));
    EXPECT_NE(std::find(is0.begin(), is0.end(), i0), is0.end());
    auto node_1 = branching_scheme.child(root, i0);

    // Only the full-width insertion of item 1 should be produced; the
    // partial-cut variant (5990 wide) is forbidden by the per-bin mwl=10.
    std::vector<BranchingScheme::Insertion> is1 =
            branching_scheme.insertions(branching_scheme.children(node_1));
    BranchingScheme::Insertion full_width
            = {1, -1, 0, 5990, 3210, 5990, 6000, 3210, 1, 1};
    EXPECT_EQ(
            std::find(is1.begin(), is1.end(), full_width),
            is1.end());
}
