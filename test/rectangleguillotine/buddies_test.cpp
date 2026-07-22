#include "packingsolver/rectangleguillotine/instance_builder.hpp"
#include "packingsolver/rectangleguillotine/optimize.hpp"
#include "rectangleguillotine/branching_scheme.hpp"
#include "rectangleguillotine/instance_flipper.hpp"
#include "rectangleguillotine/solution_builder.hpp"

#include <gtest/gtest.h>
#include <boost/filesystem.hpp>

#include <string>

using namespace packingsolver;
using namespace packingsolver::rectangleguillotine;
namespace fs = boost::filesystem;

namespace
{

// Bin 6000x3210, vertical first cut, 3 stages, NonExact — the house
// configuration shared by the building and certificate tests.
InstanceBuilder buddies_instance_builder(
        Objective objective = Objective::BinPackingWithLeftovers)
{
    InstanceBuilder instance_builder;
    instance_builder.set_objective(objective);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    return instance_builder;
}

// build() throws std::invalid_argument from ~30 distinct checks, so asserting
// only the exception *type* lets an unrelated guard satisfy a test. This pins
// the specific check by requiring a substring of its message, so each
// validation test verifies that its own check actually fired.
void expect_build_throws_with(
        InstanceBuilder& instance_builder,
        const std::string& needle)
{
    try {
        instance_builder.build();
        ADD_FAILURE() << "expected std::invalid_argument containing \""
                      << needle << "\", but build() did not throw.";
    } catch (const std::invalid_argument& e) {
        EXPECT_NE(std::string(e.what()).find(needle), std::string::npos)
                << "exception message did not contain \"" << needle
                << "\": " << e.what();
    }
}

}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////// Instance building tests ////////////////////////////
////////////////////////////////////////////////////////////////////////////////

TEST(RectangleGuillotineBuddies, BasicBuddyInstance)
{
    // Two item types sharing buddy 0 (multi-row group).
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(600, 400, -1, 2);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    EXPECT_TRUE(instance.has_buddies());
    EXPECT_EQ(instance.number_of_buddies(), 1);
    EXPECT_EQ(instance.number_of_items(), 4);
    EXPECT_EQ(instance.item_type(0).buddy_id, 0);
    EXPECT_EQ(instance.item_type(1).buddy_id, 0);
    EXPECT_EQ(instance.buddy_stacks(0).size(), 2u);
    EXPECT_EQ(instance.buddy_total(0), 4);
}

TEST(RectangleGuillotineBuddies, SingleRowMultiCopyGroup)
{
    // One row, 4 copies, all in buddy 0 — a group can be one row.
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    EXPECT_TRUE(instance.has_buddies());
    EXPECT_EQ(instance.number_of_buddies(), 1);
    EXPECT_EQ(instance.buddy_total(0), 4);
}

TEST(RectangleGuillotineBuddies, MixedBuddyAndNonBuddyItems)
{
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(600, 400, -1, 2);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(800, 300, -1, 3);   // free
    instance_builder.add_item_type(500, 200, -1, 2);   // free
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    EXPECT_TRUE(instance.has_buddies());
    EXPECT_EQ(instance.number_of_buddies(), 1);
    EXPECT_EQ(instance.number_of_items(), 9);
    EXPECT_EQ(instance.item_type(2).buddy_id, -1);
    EXPECT_EQ(instance.item_type(3).buddy_id, -1);
}

TEST(RectangleGuillotineBuddies, MultipleIndependentGroups)
{
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(600, 400, -1, 2);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(800, 300, -1, 2);
    instance_builder.set_last_item_type_buddy(1);
    instance_builder.add_item_type(700, 350, -1, 2);
    instance_builder.set_last_item_type_buddy(1);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    EXPECT_TRUE(instance.has_buddies());
    EXPECT_EQ(instance.number_of_buddies(), 2);
    EXPECT_EQ(instance.buddy_stacks(0).size(), 2u);
    EXPECT_EQ(instance.buddy_stacks(1).size(), 2u);
    EXPECT_EQ(instance.buddy_total(0), 4);
    EXPECT_EQ(instance.buddy_total(1), 4);
}

TEST(RectangleGuillotineBuddies, SparseBuddyIds)
{
    // Sparse BUDDY_IDs (50123, 99999) remapped to dense (0, 1); the
    // original CSV value is preserved on ItemType::buddy_id.
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_buddy(50123);
    instance_builder.add_item_type(600, 400, -1, 2);
    instance_builder.set_last_item_type_buddy(50123);
    instance_builder.add_item_type(800, 300, -1, 2);
    instance_builder.set_last_item_type_buddy(99999);
    instance_builder.add_item_type(700, 350, -1, 2);
    instance_builder.set_last_item_type_buddy(99999);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    EXPECT_EQ(instance.number_of_buddies(), 2);
    EXPECT_EQ(instance.item_type(0).buddy_id, 50123);
    EXPECT_EQ(instance.item_type(2).buddy_id, 99999);
    // Dense per-stack indices.
    EXPECT_EQ(instance.buddy_id_of_stack(instance.item_type(0).stack_id), 0);
    EXPECT_EQ(instance.buddy_id_of_stack(instance.item_type(2).stack_id), 1);
}

TEST(RectangleGuillotineBuddies, NonBuddyInstanceHasBuddiesIsFalse)
{
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.add_item_type(600, 400, -1, 4);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    EXPECT_FALSE(instance.has_buddies());
    EXPECT_EQ(instance.number_of_buddies(), 0);
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Validation tests ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////

TEST(RectangleGuillotineBuddies, MutualExclusionBuddyAndSetSameItem)
{
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.set_last_item_type_buddy(0);   // both set and buddy
    instance_builder.add_bin_type(6000, 3210);

    EXPECT_THROW(instance_builder.build(), std::invalid_argument);
}

TEST(RectangleGuillotineBuddies, MutualExclusionBuddyAndExplicitStackSameItem)
{
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 2, false, 0);  // stack 0
    instance_builder.set_last_item_type_buddy(0);                 // also buddy
    instance_builder.add_bin_type(6000, 3210);

    EXPECT_THROW(instance_builder.build(), std::invalid_argument);
}

TEST(RectangleGuillotineBuddies, NegativeBuddyId)
{
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_buddy(-2);  // negative but not -1
    instance_builder.add_bin_type(6000, 3210);

    expect_build_throws_with(instance_builder, "negative BUDDY_ID");
}

TEST(RectangleGuillotineBuddies, SingletonGroupRejected)
{
    // A buddy group with a single item (total 1) is meaningless.
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 1);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_bin_type(6000, 3210);

    expect_build_throws_with(instance_builder, "at least 2");
}

TEST(RectangleGuillotineBuddies, GroupTooBigByArea)
{
    // Two 3500x3210 items (each fits the bin alone) share buddy 0; their
    // combined area (22.47 M) exceeds the bin area (19.26 M), so no
    // single-plate packing can exist — rejected at build.
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(3500, 3210, -1, 1);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(3500, 3210, -1, 1);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_bin_type(6000, 3210);

    expect_build_throws_with(instance_builder, "largest usable bin area");
}

TEST(RectangleGuillotineBuddies, GroupTooBigByAreaWithTrims)
{
    // The area precheck uses the usable (trimmed) bin area: a group that
    // fits the raw plate but not the usable area is rejected. Piece
    // placement is confined to the trimmed area for every trim type, so
    // the trim type does not matter here — usable area is
    // 3210 x (6000 - 3000) = 9.63 M; the group is 2 x 2000x3210 =
    // 12.84 M > 9.63 M.
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(2000, 3210, -1, 1);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(2000, 3210, -1, 1);
    instance_builder.set_last_item_type_buddy(0);
    BinTypeId bin_type_id = instance_builder.add_bin_type(6000, 3210);
    instance_builder.add_trims(
            bin_type_id,
            3000, TrimType::Hard,   // left
            0, TrimType::Soft,      // right
            0, TrimType::Hard,      // bottom
            0, TrimType::Soft);     // top

    expect_build_throws_with(instance_builder, "largest usable bin area");
}

TEST(RectangleGuillotineBuddies, MutualExclusionStillRejectedAfterCopySupport)
{
    // The internal-copy exemption must not relax the user-input path: an
    // explicit STACK_ID + BUDDY_ID on the same item still throws.
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 2, false, 3);  // stack 3
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_bin_type(6000, 3210);

    EXPECT_THROW(instance_builder.build(), std::invalid_argument);
}

TEST(RectangleGuillotineBuddies, MutualExclusionRejectedOnBuilderReuse)
{
    // Stale exemption-registry entries from a first build must not
    // suppress the throw on a reused builder (ids restart at 0).
    InstanceBuilder source_builder = buddies_instance_builder();
    source_builder.add_item_type(1000, 500, -1, 2);
    source_builder.set_last_item_type_buddy(0);
    source_builder.add_item_type(600, 400, -1, 2);
    source_builder.set_last_item_type_buddy(0);
    source_builder.add_bin_type(6000, 3210);
    Instance source = source_builder.build();

    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(source.item_type(0), -1, 2);
    instance_builder.add_item_type(source.item_type(1), -1, 2);
    instance_builder.add_bin_type(6000, 3210);
    Instance first = instance_builder.build();
    EXPECT_TRUE(first.has_buddies());

    // Reuse: a user item with both BUDDY_ID and explicit STACK_ID lands
    // on a colliding id and must still throw.
    instance_builder.add_item_type(1000, 500, -1, 2, false, 0);
    instance_builder.set_last_item_type_buddy(0);

    EXPECT_THROW(instance_builder.build(), std::invalid_argument);
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////// Internal copy / flipper tests /////////////////////////
////////////////////////////////////////////////////////////////////////////////

TEST(RectangleGuillotineBuddies, FlipperRoundTripPropagatesBuddies)
{
    // Mixed composition; the flipped instance must carry identical stack
    // structure and buddy metadata (the branching scheme reads buddy
    // metadata from the original while pos_stack is indexed by the
    // flipped copy).
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.set_first_stage_orientation(CutOrientation::Horizontal);
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_buddy(7);
    instance_builder.add_item_type(900, 450, -1, 2, false, 0);  // explicit stack
    instance_builder.add_item_type(800, 300, -1, 3);            // free
    instance_builder.add_item_type(600, 400, -1, 2);
    instance_builder.set_last_item_type_buddy(7);
    instance_builder.add_item_type(500, 250, -1, 2);
    instance_builder.set_last_item_type_buddy(50123);           // sparse id
    instance_builder.add_item_type(450, 250, -1, 2);
    instance_builder.set_last_item_type_buddy(50123);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    ASSERT_TRUE(instance.has_buddies());
    ASSERT_EQ(instance.number_of_buddies(), 2);

    InstanceFlipper flipper(instance);
    const Instance& flipped = flipper.flipped_instance();

    EXPECT_TRUE(flipped.has_buddies());
    EXPECT_EQ(flipped.number_of_buddies(), instance.number_of_buddies());
    ASSERT_EQ(flipped.number_of_stacks(), instance.number_of_stacks());
    ASSERT_EQ(flipped.number_of_item_types(), instance.number_of_item_types());
    for (ItemTypeId item_type_id = 0;
            item_type_id < instance.number_of_item_types();
            ++item_type_id) {
        EXPECT_EQ(flipped.item_type(item_type_id).stack_id,
                  instance.item_type(item_type_id).stack_id);
        EXPECT_EQ(flipped.item_type(item_type_id).buddy_id,
                  instance.item_type(item_type_id).buddy_id);
    }
    for (StackId s = 0; s < instance.number_of_stacks(); ++s)
        EXPECT_EQ(flipped.buddy_id_of_stack(s), instance.buddy_id_of_stack(s));
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////// Branching scheme tests //////////////////////////////
////////////////////////////////////////////////////////////////////////////////

TEST(RectangleGuillotineBuddies, StackPredBuddyAxisRepairBuilds)
{
    // Identical-geometry rows across two buddy groups + a free row + a
    // set row. The two isomorphic singleton buddy groups CHAIN (strict
    // cross-group link); buddy/free and buddy/set links must break; the
    // scheme must still build. Chain/strict behavior is pinned by the
    // enforcement tests below — this only guards construction.
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_buddy(1);
    instance_builder.add_item_type(1000, 500, -1, 2);   // free, same geometry
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_set(0, 2);       // set, same geometry
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    ASSERT_TRUE(instance.has_buddies());
    ASSERT_TRUE(instance.has_sets());
    BranchingScheme branching_scheme(instance);
    EXPECT_NE(branching_scheme.root(), nullptr);
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////// Enforcement (optimize) ////////////////////////////
////////////////////////////////////////////////////////////////////////////////

namespace
{

OptimizeParameters not_anytime_parameters()
{
    OptimizeParameters optimize_parameters;
    optimize_parameters.optimization_mode
            = packingsolver::OptimizationMode::NotAnytimeSequential;
    return optimize_parameters;
}

}

TEST(RectangleGuillotineBuddies, SingleGroupCoLocates)
{
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 1);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(600, 400, -1, 1);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters = not_anytime_parameters();
    auto output = optimize(instance, optimize_parameters);

    const Solution& best = output.solution_pool.best();
    EXPECT_EQ(best.number_of_items(), 2);
    EXPECT_TRUE(best.buddies_feasible());
    EXPECT_TRUE(best.feasible());
}

TEST(RectangleGuillotineBuddies, MultiRowAndCopiesGroupsCoLocate)
{
    // One multi-row group + one single-row multi-copy group + free items.
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(600, 400, -1, 2);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(800, 700, -1, 4);   // single-row group
    instance_builder.set_last_item_type_buddy(1);
    instance_builder.add_item_type(500, 200, -1, 3);   // free
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters = not_anytime_parameters();
    auto output = optimize(instance, optimize_parameters);

    const Solution& best = output.solution_pool.best();
    EXPECT_EQ(best.number_of_items(), instance.number_of_items());
    EXPECT_TRUE(best.buddies_feasible());
    EXPECT_TRUE(best.feasible());
}

TEST(RectangleGuillotineBuddies, GroupForcedAcrossBinsDroppedUnderKnapsack)
{
    // Two 3500x2000 items share a buddy: each fits the bin alone, their
    // areas sum to 14 M < 19.26 M (so the build-time area precheck
    // passes), but no guillotine arrangement places both on one
    // 6000x3210 plate. Under the same-plate guarantee the group is
    // all-or-nothing, so it is dropped entirely; only the free filler is
    // placed. (Discriminates buddy enforcement from plain packing.)
    InstanceBuilder instance_builder = buddies_instance_builder(Objective::Knapsack);
    instance_builder.add_item_type(3500, 2000, -1, 1);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(3500, 2000, -1, 1);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(500, 500, -1, 1);   // free filler
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters = not_anytime_parameters();
    auto output = optimize(instance, optimize_parameters);

    const Solution& best = output.solution_pool.best();
    // The unplaceable-together group is dropped entirely.
    EXPECT_EQ(best.item_copies(0), 0);
    EXPECT_EQ(best.item_copies(1), 0);
    EXPECT_TRUE(best.buddies_feasible());
    // ...but the free filler (item 2, profit 250000) MUST be placed; this
    // distinguishes "group correctly dropped" from "solver returned nothing"
    // (an empty solution would also satisfy the three assertions above).
    EXPECT_EQ(best.item_copies(2), 1);
}

TEST(RectangleGuillotineBuddies, CoLocatesUnderBinPressureMultiPlate)
{
    // End-to-end co-location when MORE THAN ONE physical plate is available
    // and the group is NOT structurally confined to one plate. The earlier
    // optimize() tests (SingleGroupCoLocates, MultiRowAndCopiesGroupsCoLocate,
    // TreeSearchSolvesBuddies) all pack on a single plate, so co-location is
    // free there and the guard/certificate are never actually exercised.
    //
    // Here: 6 equal 2000x3210 items, 3 per 6000x3210 plate, 2 plates. A
    // bin-optimal 2-plate packing could split the buddy group (member A plus
    // two free on plate 0, member B plus two free on plate 1). The new-bin
    // guard + the certificate keep the two members on one plate; we assert
    // both land on the same bin index (and the job is still full).
    InstanceBuilder instance_builder = buddies_instance_builder(Objective::BinPacking);
    instance_builder.add_item_type(2000, 3210, -1, 1);   // group member A (item type 0)
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(2000, 3210, -1, 1);   // group member B (item type 1)
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(2000, 3210, -1, 4);   // 4 free fillers (item type 2)
    instance_builder.add_bin_type(6000, 3210, -1, 2);    // two physical plates
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters = not_anytime_parameters();
    auto output = optimize(instance, optimize_parameters);

    const Solution& best = output.solution_pool.best();
    EXPECT_EQ(best.number_of_items(), instance.number_of_items());  // full placement
    EXPECT_TRUE(best.feasible());
    EXPECT_TRUE(best.buddies_feasible());

    // Both group members must sit on the same physical plate (bin index).
    auto bin_of = [&](ItemTypeId item_type_id) -> int {
        for (BinPos b = 0; b < best.number_of_different_bins(); ++b)
            for (const SolutionNode& node: best.bin(b).nodes)
                if (node.item_type_id == item_type_id)
                    return (int)b;
        return -1;
    };
    int bin_a = bin_of(0);
    int bin_b = bin_of(1);
    EXPECT_NE(bin_a, -1);
    EXPECT_EQ(bin_a, bin_b);
}

////////////////////////////////////////////////////////////////////////////////
//////////////////// Isomorphic cross-group chaining (optimize) /////////////////
////////////////////////////////////////////////////////////////////////////////

namespace
{

// First distinct-bin index that holds a copy of item_type_id, or -1. Mirrors
// the inline probe in CoLocatesUnderBinPressureMultiPlate, hoisted so the
// cross-group tests below can share it.
int bin_of_item(const Solution& solution, ItemTypeId item_type_id)
{
    for (BinPos b = 0; b < solution.number_of_different_bins(); ++b)
        for (const SolutionNode& node: solution.bin(b).nodes)
            if (node.d >= 1 && node.item_type_id == item_type_id)
                return (int)b;
    return -1;
}

// Number of distinct solution bins that hold >=1 copy of item_type_id. For a
// co-located singleton buddy group (one item type, every copy on one plate)
// this is 1; a group split across plates gives >1. A buddy-bearing plate never
// collapses into a copies>1 SolutionBin (that would replicate the group across
// physical plates and fail buddies_feasible(), which every test below also
// asserts), so distinct-bin counting is a sound co-location probe here.
// The d >= 1 filter skips structural nodes (the d = 0 bin root carries the
// BIN type id in item_type_id, which collides with item type 0) — same
// filter as Solution::update_indicators' buddy pass.
int num_bins_with_item(const Solution& solution, ItemTypeId item_type_id)
{
    int count = 0;
    for (BinPos b = 0; b < solution.number_of_different_bins(); ++b) {
        for (const SolutionNode& node: solution.bin(b).nodes) {
            if (node.d >= 1 && node.item_type_id == item_type_id) {
                ++count;
                break;
            }
        }
    }
    return count;
}

}

TEST(RectangleGuillotineBuddies, IsomorphicSingletonGroupsCoLocateAtOptimum)
{
    // Four IDENTICAL 2-copy singleton buddy groups (2000x1600, non-oriented)
    // plus four free fillers of the same size. A 6000x3210 plate holds a
    // 3(wide) x 2(tall) grid = 6 pieces (3*2000=6000, 2*1600=3200<=3210), so
    // each piece is 3.2M and a plate carries 19.2M <= 19.26M usable. The 12
    // pieces need ceil(12/6)=2 plates by area and 2 plates suffice (e.g. groups
    // 0,1,2 on one plate; group 3 + four fillers on the other), so the optimum
    // is 2 bins. The four groups are equals()-identical singletons, so they are
    // cross-chained; assert the full 2-bin optimum AND that every group's two
    // copies co-locate on one plate.
    InstanceBuilder instance_builder = buddies_instance_builder(Objective::BinPacking);
    instance_builder.add_item_type(2000, 1600, -1, 2);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(2000, 1600, -1, 2);
    instance_builder.set_last_item_type_buddy(1);
    instance_builder.add_item_type(2000, 1600, -1, 2);
    instance_builder.set_last_item_type_buddy(2);
    instance_builder.add_item_type(2000, 1600, -1, 2);
    instance_builder.set_last_item_type_buddy(3);
    instance_builder.add_item_type(2000, 1600, -1, 4);   // free fillers
    instance_builder.add_bin_type(6000, 3210, -1, 2);
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters = not_anytime_parameters();
    auto output = optimize(instance, optimize_parameters);

    const Solution& best = output.solution_pool.best();
    EXPECT_EQ(best.number_of_items(), instance.number_of_items());
    EXPECT_EQ(best.number_of_bins(), 2);
    EXPECT_TRUE(best.feasible());
    EXPECT_TRUE(best.buddies_feasible());
    EXPECT_EQ(num_bins_with_item(best, 0), 1);
    EXPECT_EQ(num_bins_with_item(best, 1), 1);
    EXPECT_EQ(num_bins_with_item(best, 2), 1);
    EXPECT_EQ(num_bins_with_item(best, 3), 1);
}

TEST(RectangleGuillotineBuddies, InterleavedIsomorphicPairsReachOnePlate)
{
    // Soundness of the STRICT cross-group chain (the relabel-by-rank argument).
    // Two identical 2-copy buddy pairs (3000x1600, non-oriented). One 6000x3210
    // plate holds a 2x2 grid: two 3000-wide columns (2*3000=6000), each with
    // two 1600-tall rows (2*1600=3200<=3210). 4 pieces * 4.8M = 19.2M <= 19.26M,
    // so the sole optimum is ONE bin. The strict rule orders pair 0 fully
    // before pair 1 in INSERTION order, yet the single-plate packing is still
    // reachable: the two pieces sharing a column can always be labeled the same
    // group because the pairs are equals()-identical (relabel by rank). Two
    // bins are supplied so an over-constrained chain that lost the packing would
    // surface as a 2-bin result; assert the 1-bin optimum is found with both
    // pairs co-located.
    InstanceBuilder instance_builder = buddies_instance_builder(Objective::BinPacking);
    instance_builder.add_item_type(3000, 1600, -1, 2);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(3000, 1600, -1, 2);
    instance_builder.set_last_item_type_buddy(1);
    instance_builder.add_bin_type(6000, 3210, -1, 2);   // room to (wrongly) split
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters = not_anytime_parameters();
    auto output = optimize(instance, optimize_parameters);

    const Solution& best = output.solution_pool.best();
    EXPECT_EQ(best.number_of_items(), instance.number_of_items());  // all 4
    EXPECT_EQ(best.number_of_bins(), 1);                            // not split
    EXPECT_TRUE(best.feasible());
    EXPECT_TRUE(best.buddies_feasible());
    EXPECT_EQ(bin_of_item(best, 0), bin_of_item(best, 1));          // same plate
}

TEST(RectangleGuillotineBuddies, NonIsomorphicDifferentDimsSolveIndependently)
{
    // Two 2-copy singleton groups with DIFFERENT geometry are never chained
    // (equals() is false across them). Group A: 2x 2000x1600 (buddy 0). Group
    // B: 2x 1500x1000 (buddy 1). Areas 6.4M + 3.0M = 9.4M < 19.26M fit one
    // plate, so the optimum is 1 bin with both groups co-located. Pins the
    // negative case: differing dims must not fire the cross-group chain.
    InstanceBuilder instance_builder = buddies_instance_builder(Objective::BinPacking);
    instance_builder.add_item_type(2000, 1600, -1, 2);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(1500, 1000, -1, 2);
    instance_builder.set_last_item_type_buddy(1);
    instance_builder.add_bin_type(6000, 3210);   // single plate
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters = not_anytime_parameters();
    auto output = optimize(instance, optimize_parameters);

    const Solution& best = output.solution_pool.best();
    EXPECT_EQ(best.number_of_items(), instance.number_of_items());
    EXPECT_EQ(best.number_of_bins(), 1);
    EXPECT_TRUE(best.feasible());
    EXPECT_TRUE(best.buddies_feasible());
    EXPECT_EQ(num_bins_with_item(best, 0), 1);
    EXPECT_EQ(num_bins_with_item(best, 1), 1);
}

TEST(RectangleGuillotineBuddies, IdenticalGeometryFreeStackSplitsWhileGroupCoLocates)
{
    // A 2-copy buddy group and a FREE stack of IDENTICAL geometry. The free
    // twin must stay free to split across plates; it must never be chained to
    // (or co-located like) the buddy group. Pieces are 2900x3210 -- roughly
    // half a plate wide, full height -- so exactly two fit per 6000x3210 plate
    // (2*2900=5800<=6000, a third needs 8700>6000).
    //   - Group (item 0, buddy 0, 2 copies): both copies co-locate and fill one
    //     plate's two columns; the 200-wide remainder holds nothing more.
    //   - Free twin (item 1, 3 copies): three half-plate pieces cannot join the
    //     group's plate and cannot all share one plate, so they occupy two
    //     further plates (2 + 1) and MUST split across bins.
    // Five half-plate pieces => ceil(5/2)=3 plates optimum. Four bins are
    // supplied; assert the 3-bin optimum, all five placed (the free twin was
    // NOT dropped or over-constrained), the group co-located on one plate, and
    // the free twin genuinely spread over >=2 bins. Item 0 (buddy) precedes
    // item 1 (free) so the initial equals() link 1->0 is exactly the one the
    // buddy repair pass must break.
    InstanceBuilder instance_builder = buddies_instance_builder(Objective::BinPacking);
    instance_builder.add_item_type(2900, 3210, -1, 2);   // buddy group
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(2900, 3210, -1, 3);   // identical-geometry free twin
    instance_builder.add_bin_type(6000, 3210, -1, 4);
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters = not_anytime_parameters();
    auto output = optimize(instance, optimize_parameters);

    const Solution& best = output.solution_pool.best();
    EXPECT_EQ(best.number_of_items(), 5);            // nothing dropped
    EXPECT_EQ(best.item_copies(1), 3);               // free twin fully placed...
    EXPECT_EQ(best.number_of_bins(), 3);             // ...by splitting across bins
    EXPECT_TRUE(best.feasible());
    EXPECT_TRUE(best.buddies_feasible());
    EXPECT_EQ(num_bins_with_item(best, 0), 1);       // buddy group co-located
    EXPECT_GE(num_bins_with_item(best, 1), 2);       // free twin genuinely split
}

TEST(RectangleGuillotineBuddies, DifferentTotalsNotChained)
{
    // Two same-geometry singleton groups with DIFFERENT totals (m=2 vs m=3) are
    // not chained: the compatibility test requires equal buddy_total. Both
    // still co-locate independently. Pieces 2000x1600 (6 per plate); group 0 is
    // 2 copies, group 1 is 3 copies => 5 pieces on one plate, optimum 1 bin.
    InstanceBuilder instance_builder = buddies_instance_builder(Objective::BinPacking);
    instance_builder.add_item_type(2000, 1600, -1, 2);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(2000, 1600, -1, 3);
    instance_builder.set_last_item_type_buddy(1);
    instance_builder.add_bin_type(6000, 3210);   // single plate
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters = not_anytime_parameters();
    auto output = optimize(instance, optimize_parameters);

    const Solution& best = output.solution_pool.best();
    EXPECT_EQ(best.number_of_items(), instance.number_of_items());   // 5
    EXPECT_EQ(best.number_of_bins(), 1);
    EXPECT_TRUE(best.feasible());
    EXPECT_TRUE(best.buddies_feasible());
    EXPECT_EQ(num_bins_with_item(best, 0), 1);
    EXPECT_EQ(num_bins_with_item(best, 1), 1);
}

TEST(RectangleGuillotineBuddies, KnapsackPrefixDropKeepsOnePair)
{
    // Two identical 2-copy buddy pairs (2900x3210) but a single plate with room
    // for only ONE pair: two 2900-wide pieces fit (2*2900=5800<=6000), a third
    // does not (3*2900=8700>6000), and each pair is all-or-nothing. Under
    // Knapsack the optimum keeps exactly one full pair (profit = one pair's
    // area) and drops the other entirely -- never half a pair. Mirrors the
    // all-or-nothing style of GroupForcedAcrossBinsDroppedUnderKnapsack.
    InstanceBuilder instance_builder = buddies_instance_builder(Objective::Knapsack);
    instance_builder.add_item_type(2900, 3210, -1, 2);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(2900, 3210, -1, 2);
    instance_builder.set_last_item_type_buddy(1);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters = not_anytime_parameters();
    auto output = optimize(instance, optimize_parameters);

    const Solution& best = output.solution_pool.best();
    EXPECT_EQ(best.number_of_items(), 2);          // one full pair, no partial
    EXPECT_TRUE(best.buddies_feasible());
    // Exactly one pair retained in full, the other fully dropped (all-or-
    // nothing prefix-drop): the two copies of one buddy id, none of the other.
    bool pair0_kept = (best.item_copies(0) == 2 && best.item_copies(1) == 0);
    bool pair1_kept = (best.item_copies(0) == 0 && best.item_copies(1) == 2);
    EXPECT_TRUE(pair0_kept || pair1_kept);
}

TEST(RectangleGuillotineBuddies, IsomorphicM3GroupsCoLocateAtOptimum)
{
    // Three IDENTICAL 3-copy singleton buddy groups (2000x1600). buddy_total=3
    // for each; equal totals + singleton => they chain (m=3 chaining). 9 pieces
    // at 6 per plate need ceil(9/6)=2 plates (e.g. groups 0,1 on one plate,
    // group 2 on the other); each 3-copy group co-locates. Optimum 2 bins.
    InstanceBuilder instance_builder = buddies_instance_builder(Objective::BinPacking);
    instance_builder.add_item_type(2000, 1600, -1, 3);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(2000, 1600, -1, 3);
    instance_builder.set_last_item_type_buddy(1);
    instance_builder.add_item_type(2000, 1600, -1, 3);
    instance_builder.set_last_item_type_buddy(2);
    instance_builder.add_bin_type(6000, 3210, -1, 2);
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters = not_anytime_parameters();
    auto output = optimize(instance, optimize_parameters);

    const Solution& best = output.solution_pool.best();
    EXPECT_EQ(best.number_of_items(), instance.number_of_items());   // 9
    EXPECT_EQ(best.number_of_bins(), 2);
    EXPECT_TRUE(best.feasible());
    EXPECT_TRUE(best.buddies_feasible());
    EXPECT_EQ(num_bins_with_item(best, 0), 1);
    EXPECT_EQ(num_bins_with_item(best, 1), 1);
    EXPECT_EQ(num_bins_with_item(best, 2), 1);
}

TEST(RectangleGuillotineBuddies, ScaleManyIsomorphicGroupsSolveFast)
{
    // Regression guard for the 3^k progress-state flood. Twelve IDENTICAL
    // 2-copy singleton buddy groups (2000x1600) => 24 pieces. Without the
    // isomorphic cross-group chain these twelve interchangeable tuples yield
    // ~3^12 distinguishable progress states and tree search effectively never
    // returns; the chain collapses that to a linear number of states. Area:
    // 24 * 3.2M = 76.8M and four 19.26M plates = 77.04M (76.8M/19.26M = 3.99),
    // so the optimum is exactly 4 bins (6 pieces/plate, 3 groups/plate). Four
    // plates are supplied; assert the full 4-bin optimum is found (which, per
    // the guard, must now happen quickly rather than time out).
    InstanceBuilder instance_builder = buddies_instance_builder(Objective::BinPacking);
    for (BuddyId g = 0; g < 12; ++g) {
        instance_builder.add_item_type(2000, 1600, -1, 2);
        instance_builder.set_last_item_type_buddy(g);
    }
    instance_builder.add_bin_type(6000, 3210, -1, 4);
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters = not_anytime_parameters();
    auto output = optimize(instance, optimize_parameters);

    const Solution& best = output.solution_pool.best();
    EXPECT_EQ(best.number_of_items(), instance.number_of_items());   // all 24
    EXPECT_EQ(best.number_of_bins(), 4);
    EXPECT_TRUE(best.feasible());
    EXPECT_TRUE(best.buddies_feasible());
    for (ItemTypeId item_type_id = 0; item_type_id < 12; ++item_type_id)
        EXPECT_EQ(num_bins_with_item(best, item_type_id), 1);
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////// Solution buddy checker tests //////////////////////////
////////////////////////////////////////////////////////////////////////////////

TEST(RectangleGuillotineBuddies, CertificateCoLocatedComplete)
{
    // Multi-row group, both rows placed on the same bin → feasible.
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 1);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(1000, 500, -1, 1);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    SolutionBuilder solution_builder(instance);
    solution_builder.add_bin(0, 1, CutOrientation::Vertical);
    solution_builder.add_node(1, 1000);
    solution_builder.add_node(2, 500);
    solution_builder.set_last_node_item(0);
    solution_builder.add_node(2, 1000);
    solution_builder.set_last_node_item(1);
    Solution solution = solution_builder.build();

    EXPECT_TRUE(solution.buddies_feasible());
    EXPECT_TRUE(solution.feasible());
}

TEST(RectangleGuillotineBuddies, CertificateSplitAcrossBinsInfeasible)
{
    // The two rows of one group land on two different bins → infeasible.
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 1);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(1000, 500, -1, 1);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_bin_type(6000, 3210, -1, 2);
    Instance instance = instance_builder.build();

    SolutionBuilder solution_builder(instance);
    solution_builder.add_bin(0, 1, CutOrientation::Vertical);
    solution_builder.add_node(1, 1000);
    solution_builder.add_node(2, 500);
    solution_builder.set_last_node_item(0);
    solution_builder.add_bin(0, 1, CutOrientation::Vertical);
    solution_builder.add_node(1, 1000);
    solution_builder.add_node(2, 500);
    solution_builder.set_last_node_item(1);
    Solution solution = solution_builder.build();

    EXPECT_FALSE(solution.buddies_feasible());
    EXPECT_FALSE(solution.feasible());
}

TEST(RectangleGuillotineBuddies, CertificatePartialGroupInfeasible)
{
    // One item, 2 copies, buddy 0 (total 2). Only one copy placed →
    // partial group → infeasible (all-or-nothing).
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    SolutionBuilder solution_builder(instance);
    solution_builder.add_bin(0, 1, CutOrientation::Vertical);
    solution_builder.add_node(1, 1000);
    solution_builder.add_node(2, 500);
    solution_builder.set_last_node_item(0);   // only 1 of 2
    Solution solution = solution_builder.build();

    EXPECT_FALSE(solution.buddies_feasible());
    // A partial group must also make the solution infeasible overall, so the
    // comparison (operator<) cannot prefer it (feasible() folds this in).
    EXPECT_FALSE(solution.feasible());
}

TEST(RectangleGuillotineBuddies, CertificateReplicatedHostInfeasible)
{
    // One item, 2 copies, buddy 0. Placed once in a bin replicated twice
    // (copies = 2): the count matches the total, but the two pieces sit
    // on two different physical plates → infeasible.
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_bin_type(6000, 3210, -1, 2);
    Instance instance = instance_builder.build();

    SolutionBuilder solution_builder(instance);
    solution_builder.add_bin(0, 2, CutOrientation::Vertical);
    solution_builder.add_node(1, 1000);
    solution_builder.add_node(2, 500);
    solution_builder.set_last_node_item(0);
    Solution solution = solution_builder.build();

    EXPECT_FALSE(solution.buddies_feasible());
    EXPECT_FALSE(solution.feasible());
}

TEST(RectangleGuillotineBuddies, CertificateCopiesItemCoLocatedFeasible)
{
    // One item, 2 copies, buddy 0, both copies on a single (copies == 1)
    // bin → feasible (the positive copies>1-item-on-one-plate case).
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    SolutionBuilder solution_builder(instance);
    solution_builder.add_bin(0, 1, CutOrientation::Vertical);
    solution_builder.add_node(1, 1000);
    solution_builder.add_node(2, 500);
    solution_builder.set_last_node_item(0);
    solution_builder.add_node(2, 1000);
    solution_builder.set_last_node_item(0);
    Solution solution = solution_builder.build();

    EXPECT_TRUE(solution.buddies_feasible());
    EXPECT_TRUE(solution.feasible());
}

TEST(RectangleGuillotineBuddies, CertificateUntouchedGroupFeasible)
{
    // A group placed zero times is feasible (all-or-nothing permits
    // none) — here a free item is placed and the group is left out.
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(800, 300, -1, 1);   // free
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    SolutionBuilder solution_builder(instance);
    solution_builder.add_bin(0, 1, CutOrientation::Vertical);
    solution_builder.add_node(1, 800);
    solution_builder.add_node(2, 300);
    solution_builder.set_last_node_item(1);   // free item only
    Solution solution = solution_builder.build();

    EXPECT_TRUE(solution.buddies_feasible());
}

TEST(RectangleGuillotineBuddies, CertificateSparseIdsDenseIndexing)
{
    // Sparse buddy ids: the checker state is sized number_of_buddies()
    // (dense) and must index through buddy_id_of_stack(), never through
    // the sparse ItemType::buddy_id. Group 99999 split across two bins.
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 1);
    instance_builder.set_last_item_type_buddy(50123);
    instance_builder.add_item_type(1000, 500, -1, 1);
    instance_builder.set_last_item_type_buddy(50123);
    instance_builder.add_item_type(800, 300, -1, 1);
    instance_builder.set_last_item_type_buddy(99999);
    instance_builder.add_item_type(800, 300, -1, 1);
    instance_builder.set_last_item_type_buddy(99999);
    instance_builder.add_bin_type(6000, 3210, -1, 2);
    Instance instance = instance_builder.build();

    SolutionBuilder solution_builder(instance);
    // Group 50123 (items 0,1) co-located on bin 0.
    solution_builder.add_bin(0, 1, CutOrientation::Vertical);
    solution_builder.add_node(1, 1000);
    solution_builder.add_node(2, 500);
    solution_builder.set_last_node_item(0);
    solution_builder.add_node(2, 1000);
    solution_builder.set_last_node_item(1);
    // Group 99999 (items 2,3) split: item 2 on bin 0, item 3 on bin 1.
    solution_builder.add_node(1, 1800);
    solution_builder.add_node(2, 300);
    solution_builder.set_last_node_item(2);
    solution_builder.add_bin(0, 1, CutOrientation::Vertical);
    solution_builder.add_node(1, 800);
    solution_builder.add_node(2, 300);
    solution_builder.set_last_node_item(3);
    Solution solution = solution_builder.build();

    // Group 99999 spans two bins → infeasible.
    EXPECT_FALSE(solution.buddies_feasible());
}

TEST(RectangleGuillotineBuddies, CertificateSparseIdCoLocatedFeasible)
{
    // Positive companion to CertificateSparseIdsDenseIndexing: a single group
    // with a large sparse id (99999), fully co-located on one bin. The checker
    // sizes its state by number_of_buddies() and must index through the dense
    // buddy_id_of_stack(); asserting feasible here pins that dense mapping (a
    // naive buddy_bins_[99999] would be out of range, never feasible), so the
    // suite no longer relies only on the negative (spans-two-bins) case.
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 1);
    instance_builder.set_last_item_type_buddy(99999);
    instance_builder.add_item_type(1000, 500, -1, 1);
    instance_builder.set_last_item_type_buddy(99999);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    SolutionBuilder solution_builder(instance);
    solution_builder.add_bin(0, 1, CutOrientation::Vertical);
    solution_builder.add_node(1, 1000);
    solution_builder.add_node(2, 500);
    solution_builder.set_last_node_item(0);
    solution_builder.add_node(2, 1000);
    solution_builder.set_last_node_item(1);
    Solution solution = solution_builder.build();

    EXPECT_TRUE(solution.buddies_feasible());
    EXPECT_TRUE(solution.feasible());
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////// Algorithm selection tests ////////////////////////////
////////////////////////////////////////////////////////////////////////////////

TEST(RectangleGuillotineBuddies, MultiBinVbppRejected)
{
    // Buddies + variable-sized bin packing with more than one bin type:
    // tree search (required for the guarantee) is unavailable there, so
    // optimize must reject the instance.
    InstanceBuilder instance_builder = buddies_instance_builder(
            Objective::VariableSizedBinPacking);
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(600, 400, -1, 2);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_bin_type(6000, 3210, -1, 5);
    instance_builder.add_bin_type(4000, 3000, -1, 5);   // second bin type
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters = not_anytime_parameters();
    EXPECT_THROW(optimize(instance, optimize_parameters),
                 std::invalid_argument);
}

TEST(RectangleGuillotineBuddies, ExplicitColumnGenerationRejected)
{
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(600, 400, -1, 2);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters = not_anytime_parameters();
    optimize_parameters.use_column_generation = true;
    EXPECT_THROW(optimize(instance, optimize_parameters),
                 std::invalid_argument);
}

TEST(RectangleGuillotineBuddies, ExplicitSvcRejected)
{
    // SSK/SVC/DS are not supported for buddies in v1.
    InstanceBuilder instance_builder = buddies_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(600, 400, -1, 2);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters = not_anytime_parameters();
    optimize_parameters.use_sequential_value_correction = true;
    EXPECT_THROW(optimize(instance, optimize_parameters),
                 std::invalid_argument);
}

TEST(RectangleGuillotineBuddies, TreeSearchSolvesBuddies)
{
    // Default auto-selection on a buddy instance runs tree search and
    // produces a co-located full placement.
    InstanceBuilder instance_builder = buddies_instance_builder(Objective::BinPacking);
    instance_builder.add_item_type(1500, 1000, -1, 4);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(1200, 800, -1, 4);
    instance_builder.set_last_item_type_buddy(0);
    instance_builder.add_item_type(1000, 600, -1, 6);   // free
    instance_builder.add_bin_type(6000, 3210, -1, 5);
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters = not_anytime_parameters();
    auto output = optimize(instance, optimize_parameters);

    const Solution& best = output.solution_pool.best();
    EXPECT_EQ(best.number_of_items(), instance.number_of_items());
    EXPECT_TRUE(best.buddies_feasible());
    EXPECT_TRUE(best.feasible());
}
