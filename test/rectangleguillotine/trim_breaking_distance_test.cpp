#include "packingsolver/rectangleguillotine/instance_builder.hpp"
#include "packingsolver/rectangleguillotine/optimize.hpp"

#include <gtest/gtest.h>

using namespace packingsolver;
using namespace packingsolver::rectangleguillotine;

namespace
{

OptimizeParameters not_anytime_parameters()
{
    OptimizeParameters optimize_parameters;
    optimize_parameters.optimization_mode
            = packingsolver::OptimizationMode::NotAnytimeSequential;
    return optimize_parameters;
}

// 3-stage, NonExact, BinPackingWithLeftovers — the house configuration.
InstanceBuilder bd_trim_instance_builder()
{
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    return instance_builder;
}

// True if some waste node (item_type_id == -1) sits against x == 0 with exactly
// the given width — i.e. the left trim band landed on the x-axis at this size.
bool has_left_band(const Solution& solution, Length width)
{
    for (const SolutionNode& node: solution.bin(0).nodes)
        if (node.item_type_id == -1 && node.l == 0 && node.r - node.l == width)
            return true;
    return false;
}

// True if some waste node sits against y == 0 with exactly the given height —
// i.e. the bottom trim band landed on the y-axis at this size.
bool has_bottom_band(const Solution& solution, Length height)
{
    for (const SolutionNode& node: solution.bin(0).nodes)
        if (node.item_type_id == -1 && node.b == 0 && node.t - node.b == height)
            return true;
    return false;
}

// True if some usable waste node abuts the soft LEFT trim (its near edge sits on
// the trim line, node.l == left_trim) with width strictly less than min_waste —
// i.e. a sub-mwl sliver that is legal ONLY via the adjacent discount. Its
// presence in a FEASIBLE solution proves the discount branch is load-bearing.
bool has_adjacent_left_subwaste(const Solution& solution, Length left_trim, Length min_waste)
{
    for (const SolutionNode& node: solution.bin(0).nodes)
        if (node.item_type_id == -1
                && node.l == left_trim
                && node.r - node.l < min_waste)
            return true;
    return false;
}

}

////////////////////////////////////////////////////////////////////////////////
// Issue #1 — breaking distance (minimum_waste_length) > soft trim.
// The soft-trim band (width = trim - cut_thickness) is shorter than the
// breaking distance; before the fix it tripped the mwl feasibility check and
// the binary called exit(1).
////////////////////////////////////////////////////////////////////////////////

TEST(RectangleGuillotineBreakingDistanceTrim, BreakingDistanceExceedsSoftTrim)
{
    InstanceBuilder instance_builder = bd_trim_instance_builder();
    instance_builder.set_minimum_waste_length(20);
    BinTypeId bin_type_id = instance_builder.add_bin_type(6000, 3210);
    instance_builder.add_trims(
            bin_type_id,
            10, TrimType::Soft,   // left
            10, TrimType::Soft,   // right
            10, TrimType::Soft,   // bottom
            10, TrimType::Soft);  // top
    instance_builder.add_item_type(1800, 3000, -1, 3, true);
    Instance instance = instance_builder.build();

    auto output = optimize(instance, not_anytime_parameters());
    const Solution& best = output.solution_pool.best();

    EXPECT_TRUE(best.feasible());
    EXPECT_TRUE(best.minimum_waste_length_feasible());
    EXPECT_EQ(best.number_of_items(), 3);
    EXPECT_EQ(best.number_of_bins(), 1);
}

TEST(RectangleGuillotineBreakingDistanceTrim, BreakingDistanceEqualsSoftTrim)
{
    // Boundary: mwl == trim has always worked; keep it working.
    InstanceBuilder instance_builder = bd_trim_instance_builder();
    instance_builder.set_minimum_waste_length(10);
    BinTypeId bin_type_id = instance_builder.add_bin_type(6000, 3210);
    instance_builder.add_trims(
            bin_type_id,
            10, TrimType::Soft, 10, TrimType::Soft,
            10, TrimType::Soft, 10, TrimType::Soft);
    instance_builder.add_item_type(1800, 3000, -1, 3, true);
    Instance instance = instance_builder.build();

    auto output = optimize(instance, not_anytime_parameters());
    const Solution& best = output.solution_pool.best();
    EXPECT_TRUE(best.feasible());
    EXPECT_TRUE(best.minimum_waste_length_feasible());
    EXPECT_EQ(best.number_of_bins(), 1);
}

////////////////////////////////////////////////////////////////////////////////
// Issue #2 — asymmetric (left != bottom) soft trims under horizontal / any.
// to_solution() emits left_trim at depth-1 and bottom_trim at depth-2; under a
// horizontal first cut depth->axis is swapped, so before the fix the trims
// landed on swapped axes. The fix makes the emission orientation-aware; the
// checker keys on node coordinates so the final geometry is identical to the
// vertical case.
////////////////////////////////////////////////////////////////////////////////

TEST(RectangleGuillotineBreakingDistanceTrim, AsymmetricSoftTrimsHorizontal)
{
    InstanceBuilder instance_builder = bd_trim_instance_builder();
    instance_builder.set_minimum_waste_length(20);
    instance_builder.set_first_stage_orientation(CutOrientation::Horizontal);
    BinTypeId bin_type_id = instance_builder.add_bin_type(6000, 3210);
    instance_builder.add_trims(
            bin_type_id,
            10, TrimType::Soft,   // left
            10, TrimType::Soft,   // right
            30, TrimType::Soft,   // bottom
            30, TrimType::Soft);  // top
    instance_builder.add_item_type(1800, 3000, -1, 3, true);
    Instance instance = instance_builder.build();

    auto output = optimize(instance, not_anytime_parameters());
    const Solution& best = output.solution_pool.best();

    EXPECT_TRUE(best.feasible());
    EXPECT_TRUE(best.minimum_waste_length_feasible());
    EXPECT_EQ(best.number_of_bins(), 1);
    // Left trim (10) on the x-axis, bottom trim (30) on the y-axis — NOT swapped.
    EXPECT_TRUE(has_left_band(best, 10));
    EXPECT_TRUE(has_bottom_band(best, 30));
}

TEST(RectangleGuillotineBreakingDistanceTrim, AsymmetricSoftTrimsVerticalGeometry)
{
    // Same instance, vertical first cut — geometry must match the horizontal case.
    InstanceBuilder instance_builder = bd_trim_instance_builder();
    instance_builder.set_minimum_waste_length(20);
    instance_builder.set_first_stage_orientation(CutOrientation::Vertical);
    BinTypeId bin_type_id = instance_builder.add_bin_type(6000, 3210);
    instance_builder.add_trims(
            bin_type_id,
            10, TrimType::Soft, 10, TrimType::Soft,
            30, TrimType::Soft, 30, TrimType::Soft);
    instance_builder.add_item_type(1800, 3000, -1, 3, true);
    Instance instance = instance_builder.build();

    auto output = optimize(instance, not_anytime_parameters());
    const Solution& best = output.solution_pool.best();
    EXPECT_TRUE(best.feasible());
    EXPECT_TRUE(has_left_band(best, 10));
    EXPECT_TRUE(has_bottom_band(best, 30));
}

TEST(RectangleGuillotineBreakingDistanceTrim, FourDistinctSoftTrimsAny)
{
    // Four distinct soft trims, orientation 'any', mwl between the smallest and
    // largest trim. Must not crash and must stay feasible under both branches.
    InstanceBuilder instance_builder = bd_trim_instance_builder();
    instance_builder.set_minimum_waste_length(15);
    instance_builder.set_first_stage_orientation(CutOrientation::Any);
    BinTypeId bin_type_id = instance_builder.add_bin_type(6000, 3210);
    instance_builder.add_trims(
            bin_type_id,
            10, TrimType::Soft,   // left
            30, TrimType::Soft,   // right
            20, TrimType::Soft,   // bottom
            40, TrimType::Soft);  // top
    instance_builder.add_item_type(1800, 2900, -1, 3, true);
    Instance instance = instance_builder.build();

    auto output = optimize(instance, not_anytime_parameters());
    const Solution& best = output.solution_pool.best();
    EXPECT_TRUE(best.feasible());
    EXPECT_TRUE(best.minimum_waste_length_feasible());
    EXPECT_EQ(best.number_of_items(), 3);
}

TEST(RectangleGuillotineBreakingDistanceTrim, AsymmetricSoftTrimsHorizontalCutThickness)
{
    // cut_thickness > 0 with asymmetric soft trims under horizontal: the band
    // width is (trim - cut_thickness); the exemption must still apply.
    InstanceBuilder instance_builder = bd_trim_instance_builder();
    instance_builder.set_minimum_waste_length(18);
    instance_builder.set_cut_thickness(3);
    instance_builder.set_first_stage_orientation(CutOrientation::Horizontal);
    BinTypeId bin_type_id = instance_builder.add_bin_type(6000, 3210);
    instance_builder.add_trims(
            bin_type_id,
            12, TrimType::Soft, 12, TrimType::Soft,
            28, TrimType::Soft, 28, TrimType::Soft);
    instance_builder.add_item_type(1800, 3000, -1, 3, true);
    Instance instance = instance_builder.build();

    auto output = optimize(instance, not_anytime_parameters());
    const Solution& best = output.solution_pool.best();
    EXPECT_TRUE(best.feasible());
    EXPECT_TRUE(best.minimum_waste_length_feasible());
    EXPECT_TRUE(has_left_band(best, 12 - 3));
    EXPECT_TRUE(has_bottom_band(best, 28 - 3));
}

////////////////////////////////////////////////////////////////////////////////
// Negative — the checker must still reject genuine sub-mwl waste. These pin the
// non-over-relaxation property on both the baseline (no-trim) path and the new
// soft-trim path, so a future over-relaxation (e.g. dropping the std::max clamp
// or widening the discount) is caught.
////////////////////////////////////////////////////////////////////////////////

TEST(RectangleGuillotineBreakingDistanceTrim, SubMwlWasteRejectedBaselineNoTrim)
{
    // Baseline (NO trim, so the soft-trim exempt/discount branches are inert and
    // this exercises the unchanged full-min_waste path): 3 items of width 1999
    // (total 5997) in a 6000-wide bin leave only 3mm of trailing edge waste < mwl
    // 22. The solver must NOT accept the sub-mwl sliver; it spills to a 2nd bin.
    InstanceBuilder instance_builder = bd_trim_instance_builder();
    instance_builder.set_minimum_waste_length(22);
    instance_builder.add_bin_type(6000, 3210, -1, 10);  // up to 10 bins
    instance_builder.add_item_type(1999, 3000, -1, 3, true);
    Instance instance = instance_builder.build();

    auto output = optimize(instance, not_anytime_parameters());
    const Solution& best = output.solution_pool.best();
    EXPECT_TRUE(best.feasible());
    EXPECT_TRUE(best.minimum_waste_length_feasible());
    EXPECT_GE(best.number_of_bins(), 2);
}

TEST(RectangleGuillotineBreakingDistanceTrim, SubMwlWasteRejectedWithSoftTrimPresent)
{
    // A soft LEFT trim IS present, but the sub-mwl waste is NOT adjacent to it:
    // 3 items of width 1993 in a bin with left_trim=10 (usable 5990) leave 11mm
    // trailing waste against the right edge. Whether the solver left-justifies
    // (trailing waste at node.l != left_trim) or shifts to abut the trim (11mm at
    // node.l == left_trim, which the discount only allows at >= mwl-trim = 12),
    // 11 < 12 so it is rejected either way -> 2 bins. Proves the discount does not
    // over-relax: a sub-mwl sliver below the discounted threshold is still refused.
    InstanceBuilder instance_builder = bd_trim_instance_builder();
    instance_builder.set_minimum_waste_length(22);
    BinTypeId bin_type_id = instance_builder.add_bin_type(6000, 3210, -1, 10);
    instance_builder.add_trims(
            bin_type_id,
            10, TrimType::Soft, 0, TrimType::Soft,
            0, TrimType::Soft, 0, TrimType::Soft);
    instance_builder.add_item_type(1993, 3000, -1, 3, true);
    Instance instance = instance_builder.build();

    auto output = optimize(instance, not_anytime_parameters());
    const Solution& best = output.solution_pool.best();
    EXPECT_TRUE(best.feasible());
    EXPECT_TRUE(best.minimum_waste_length_feasible());
    EXPECT_GE(best.number_of_bins(), 2);
}

TEST(RectangleGuillotineBreakingDistanceTrim, SoftTrimAdjacentWasteDiscounted)
{
    // Exercises the adjacent-DISCOUNT branch (the half a mutation-test showed was
    // otherwise uncovered). A full-height defect blocking x in [left_trim, mwl]
    // forces all content to start at x == mwl, leaving a usable waste strip
    // [left_trim, mwl] of width (mwl - left_trim) abutting the soft left trim.
    // That strip is < mwl but legal: trim + strip == mwl forms one contiguous
    // breakable border (the search produces exactly this via its own soft-trim
    // defect discount). Removing the checker discount makes this infeasible.
    const Length left_trim = 10, mwl = 20;
    InstanceBuilder instance_builder = bd_trim_instance_builder();
    instance_builder.set_minimum_waste_length(mwl);
    BinTypeId bin_type_id = instance_builder.add_bin_type(6000, 3210);
    instance_builder.add_trims(
            bin_type_id,
            left_trim, TrimType::Soft, 0, TrimType::Soft,
            0, TrimType::Soft, 0, TrimType::Soft);
    instance_builder.add_defect(bin_type_id, left_trim, 0, mwl - left_trim, 3210);
    instance_builder.add_item_type(1800, 3000, -1, 3, true);
    Instance instance = instance_builder.build();

    auto output = optimize(instance, not_anytime_parameters());
    const Solution& best = output.solution_pool.best();
    EXPECT_TRUE(best.feasible());
    EXPECT_TRUE(best.minimum_waste_length_feasible());
    EXPECT_EQ(best.number_of_bins(), 1);
    // The trim-adjacent sub-mwl strip must be present in the feasible solution,
    // which is only possible because the discount accepted it.
    EXPECT_TRUE(has_adjacent_left_subwaste(best, left_trim, mwl));
}

// NOTE on the discount's lower bound: there is intentionally no test asserting
// that a *sub-threshold* trim-adjacent waste is rejected, because the search
// cannot produce one. The search-side discount (branching_scheme.cpp ~1157-1162)
// already places content at x == min_waste when abutting a soft trim, so the
// adjacent waste it emits is always exactly (min_waste - trim) — never less.
// The checker's discount is a validation MIRROR of that: its job is to AGREE
// with the search (accept exactly-threshold waste, which SoftTrimAdjacentWaste-
// Discounted covers) rather than to reject smaller waste that never occurs.
// Consequently a mutation that over-relaxes the discount (e.g. min_waste_x = 0)
// is an equivalent mutant at the behavioral level — it cannot change any
// optimize() outcome. What IS observable, and what the positive test guards, is
// the discount's PRESENCE: removing it makes the checker reject the search's own
// legitimate threshold-waste, reintroducing the Issue #1 crash for trimmed jobs.

////////////////////////////////////////////////////////////////////////////////
// No-regression — Hard trims and right/top soft trims are deliberately untouched.
////////////////////////////////////////////////////////////////////////////////

TEST(RectangleGuillotineBreakingDistanceTrim, BreakingDistanceExceedsHardTrim)
{
    // Hard trims reconstruct at depth d==-1 and never reach the d>=1 mwl check,
    // so mwl > hard trim has always been fine. Pin it: the soft-trim fix must not
    // change this (the exempt/discount branches gate on TrimType::Soft).
    InstanceBuilder instance_builder = bd_trim_instance_builder();
    instance_builder.set_minimum_waste_length(20);
    BinTypeId bin_type_id = instance_builder.add_bin_type(6000, 3210);
    instance_builder.add_trims(
            bin_type_id,
            10, TrimType::Hard, 10, TrimType::Hard,
            10, TrimType::Hard, 10, TrimType::Hard);
    instance_builder.add_item_type(1800, 3000, -1, 3, true);
    Instance instance = instance_builder.build();

    auto output = optimize(instance, not_anytime_parameters());
    const Solution& best = output.solution_pool.best();
    EXPECT_TRUE(best.feasible());
    EXPECT_TRUE(best.minimum_waste_length_feasible());
    EXPECT_EQ(best.number_of_items(), 3);
    EXPECT_EQ(best.number_of_bins(), 1);
}

TEST(RectangleGuillotineBreakingDistanceTrim, RightTopSoftTrimsNoLeftBottomBand)
{
    // Only right/top soft trims (left/bottom = 0), mwl between. Right/top soft
    // trims are absorbed into the trailing leftover, NOT emitted as bands, so the
    // checker's left/bottom-only scope is correct here: no node abuts x==0 or
    // y==0 as a sub-mwl band, and the job stays feasible.
    InstanceBuilder instance_builder = bd_trim_instance_builder();
    instance_builder.set_minimum_waste_length(20);
    BinTypeId bin_type_id = instance_builder.add_bin_type(6000, 3210);
    instance_builder.add_trims(
            bin_type_id,
            0, TrimType::Soft, 30, TrimType::Soft,
            0, TrimType::Soft, 30, TrimType::Soft);
    instance_builder.add_item_type(1800, 3000, -1, 3, true);
    Instance instance = instance_builder.build();

    auto output = optimize(instance, not_anytime_parameters());
    const Solution& best = output.solution_pool.best();
    EXPECT_TRUE(best.feasible());
    EXPECT_TRUE(best.minimum_waste_length_feasible());
    EXPECT_EQ(best.number_of_items(), 3);
}
