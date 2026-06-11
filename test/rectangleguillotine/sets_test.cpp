#include "packingsolver/rectangleguillotine/instance_builder.hpp"
#include "packingsolver/rectangleguillotine/optimize.hpp"
#include "rectangleguillotine/branching_scheme.hpp"
#include "rectangleguillotine/instance_flipper.hpp"
#include "rectangleguillotine/solution_builder.hpp"

#include <gtest/gtest.h>
#include <boost/filesystem.hpp>

using namespace packingsolver;
using namespace packingsolver::rectangleguillotine;
namespace fs = boost::filesystem;

////////////////////////////////////////////////////////////////////////////////
/////////////////////////// Instance building tests ////////////////////////////
////////////////////////////////////////////////////////////////////////////////

TEST(RectangleGuillotineSets, BasicSetInstance)
{
    // Two item types in set 0 with SET_SIZE=2.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_item_type(600, 400, -1, 4);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    EXPECT_TRUE(instance.has_sets());
    EXPECT_EQ(instance.number_of_sets(), 1);
    EXPECT_EQ(instance.number_of_items(), 8);
    EXPECT_EQ(instance.item_type(0).set_id, 0);
    EXPECT_EQ(instance.item_type(0).set_size, 2);
    EXPECT_EQ(instance.item_type(1).set_id, 0);
    EXPECT_EQ(instance.item_type(1).set_size, 2);
}

TEST(RectangleGuillotineSets, MixedSetAndNonSetItems)
{
    // Two set items + two non-set items.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_item_type(600, 400, -1, 4);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_item_type(800, 300, -1, 3);
    instance_builder.add_item_type(500, 200, -1, 2);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    EXPECT_TRUE(instance.has_sets());
    EXPECT_EQ(instance.number_of_sets(), 1);
    EXPECT_EQ(instance.number_of_items(), 13);
    // Non-set items retain default set_id/set_size.
    EXPECT_EQ(instance.item_type(2).set_id, -1);
    EXPECT_EQ(instance.item_type(2).set_size, -1);
    EXPECT_EQ(instance.item_type(3).set_id, -1);
}

TEST(RectangleGuillotineSets, MultipleIndependentSets)
{
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_item_type(600, 400, -1, 4);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(1, 2);
    instance_builder.add_item_type(800, 600, -1, 6);
    instance_builder.set_last_item_type_set(1, 3);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    EXPECT_TRUE(instance.has_sets());
    EXPECT_EQ(instance.number_of_sets(), 2);
    EXPECT_EQ(instance.set_stacks(0).size(), 2u);
    EXPECT_EQ(instance.set_stacks(1).size(), 2u);
}

TEST(RectangleGuillotineSets, SparseSetIds)
{
    // Sparse SET_IDs (50123, 99999) should be remapped to dense (0, 1).
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(50123, 2);
    instance_builder.add_item_type(600, 400, -1, 4);
    instance_builder.set_last_item_type_set(50123, 2);
    instance_builder.add_item_type(800, 300, -1, 6);
    instance_builder.set_last_item_type_set(99999, 3);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    EXPECT_EQ(instance.number_of_sets(), 2);
    // Original SET_IDs preserved on ItemType.
    EXPECT_EQ(instance.item_type(0).set_id, 50123);
    EXPECT_EQ(instance.item_type(2).set_id, 99999);
}

TEST(RectangleGuillotineSets, NonSetInstanceHasSetsIsFalse)
{
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.add_item_type(600, 400, -1, 4);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    EXPECT_FALSE(instance.has_sets());
    EXPECT_EQ(instance.number_of_sets(), 0);
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Validation tests ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////

TEST(RectangleGuillotineSets, MutualExclusionSetAndStackSameItem)
{
    // Same item has both explicit stack_id and set_id — rejected.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4, false, 0);  // explicit stack_id=0
    instance_builder.set_last_item_type_set(0, 2);                // also set on same item
    instance_builder.add_bin_type(6000, 3210);

    EXPECT_THROW(instance_builder.build(), std::invalid_argument);
}

TEST(RectangleGuillotineSets, MixedSetAndExplicitStackBuilds)
{
    // One item with explicit stack_id, another with set_id — allowed.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4, false, 0);  // explicit stack_id=0
    instance_builder.add_item_type(600, 400, -1, 4);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    EXPECT_TRUE(instance.has_sets());
    EXPECT_EQ(instance.number_of_sets(), 1);
    // Stack item: stack_id=0, no set
    EXPECT_EQ(instance.item_type(0).set_id, -1);
    // Set item: auto-assigned stack, set_id=0
    EXPECT_EQ(instance.item_type(1).set_id, 0);
}

TEST(RectangleGuillotineSets, CopiesNotDivisibleBySetSize)
{
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 5);  // 5 not divisible by 2
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_bin_type(6000, 3210);

    EXPECT_THROW(instance_builder.build(), std::invalid_argument);
}

TEST(RectangleGuillotineSets, SetSizeWithoutSetId)
{
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(0, 2);  // valid set item
    instance_builder.add_item_type(600, 400, -1, 4);
    instance_builder.set_last_item_type_set(-1, 2);  // SET_SIZE without SET_ID
    instance_builder.add_bin_type(6000, 3210);

    EXPECT_THROW(instance_builder.build(), std::invalid_argument);
}

TEST(RectangleGuillotineSets, OrphanSetSizeNoSetAnywhere)
{
    // SET_SIZE set but no SET_ID anywhere in the instance.
    // The unconditional orphan check should catch this.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(-1, 2);
    instance_builder.add_bin_type(6000, 3210);

    EXPECT_THROW(instance_builder.build(), std::invalid_argument);
}

TEST(RectangleGuillotineSets, NegativeSetId)
{
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(-2, 2);  // negative but not -1
    instance_builder.add_bin_type(6000, 3210);

    EXPECT_THROW(instance_builder.build(), std::invalid_argument);
}

TEST(RectangleGuillotineSets, MissingSetSize)
{
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(0, -1);  // SET_ID without SET_SIZE
    instance_builder.add_bin_type(6000, 3210);

    EXPECT_THROW(instance_builder.build(), std::invalid_argument);
}

TEST(RectangleGuillotineSets, SetSizeZero)
{
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(0, 0);  // SET_SIZE=0 invalid
    instance_builder.add_bin_type(6000, 3210);

    EXPECT_THROW(instance_builder.build(), std::invalid_argument);
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////// CSV parsing tests /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

TEST(RectangleGuillotineSets, CsvBasicSet)
{
    InstanceBuilder instance_builder;
    instance_builder.read_item_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_basic" / "items.csv").string());
    instance_builder.read_bin_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_basic" / "bins.csv").string());
    instance_builder.read_parameters(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_basic" / "parameters.csv").string());
    Instance instance = instance_builder.build();

    EXPECT_TRUE(instance.has_sets());
    EXPECT_EQ(instance.number_of_sets(), 1);
    EXPECT_EQ(instance.number_of_items(), 8);
    EXPECT_EQ(instance.item_type(0).set_id, 0);
    EXPECT_EQ(instance.item_type(0).set_size, 2);
}

TEST(RectangleGuillotineSets, CsvMixedSetAndNonSet)
{
    InstanceBuilder instance_builder;
    instance_builder.read_item_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_mixed" / "items.csv").string());
    instance_builder.read_bin_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_mixed" / "bins.csv").string());
    instance_builder.read_parameters(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_mixed" / "parameters.csv").string());
    Instance instance = instance_builder.build();

    EXPECT_TRUE(instance.has_sets());
    EXPECT_EQ(instance.number_of_items(), 13);
    EXPECT_EQ(instance.item_type(2).set_id, -1);
}

TEST(RectangleGuillotineSets, CsvMultipleSets)
{
    InstanceBuilder instance_builder;
    instance_builder.read_item_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_multiple" / "items.csv").string());
    instance_builder.read_bin_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_multiple" / "bins.csv").string());
    instance_builder.read_parameters(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_multiple" / "parameters.csv").string());
    Instance instance = instance_builder.build();

    EXPECT_TRUE(instance.has_sets());
    EXPECT_EQ(instance.number_of_sets(), 2);
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////// Internal copy / flipper tests /////////////////////////
////////////////////////////////////////////////////////////////////////////////

TEST(RectangleGuillotineSets, FlipperRoundTripPropagatesSets)
{
    // Mixed composition: set items interleaved with implicit-stack non-set
    // items and explicit-stack items. The flipped instance must have an
    // IDENTICAL stack structure (count + per-item-type stack_id) and
    // identical set metadata — the branching scheme reads set metadata from
    // the original instance while pos_stack is indexed by the flipped one,
    // so any divergence means wrong enforcement or out-of-bounds.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.set_first_stage_orientation(CutOrientation::Horizontal);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(7, 2);               // set item
    instance_builder.add_item_type(900, 450, -1, 2, false, 0);  // explicit stack 0
    instance_builder.add_item_type(800, 300, -1, 3);             // implicit non-set
    instance_builder.add_item_type(600, 400, -1, 2);
    instance_builder.set_last_item_type_set(7, 2);               // set item, same set
    instance_builder.add_item_type(700, 350, -1, 1, false, 0);  // explicit stack 0
    instance_builder.add_item_type(500, 250, -1, 3);
    instance_builder.set_last_item_type_set(50123, 3);           // sparse set id
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    ASSERT_TRUE(instance.has_sets());
    ASSERT_EQ(instance.number_of_sets(), 2);

    InstanceFlipper flipper(instance);
    const Instance& flipped = flipper.flipped_instance();

    EXPECT_TRUE(flipped.has_sets());
    EXPECT_EQ(flipped.number_of_sets(), instance.number_of_sets());
    ASSERT_EQ(flipped.number_of_stacks(), instance.number_of_stacks());
    ASSERT_EQ(flipped.number_of_item_types(), instance.number_of_item_types());
    for (ItemTypeId item_type_id = 0;
            item_type_id < instance.number_of_item_types();
            ++item_type_id) {
        EXPECT_EQ(flipped.item_type(item_type_id).stack_id,
                  instance.item_type(item_type_id).stack_id);
        EXPECT_EQ(flipped.item_type(item_type_id).stack_pos,
                  instance.item_type(item_type_id).stack_pos);
        EXPECT_EQ(flipped.item_type(item_type_id).set_id,
                  instance.item_type(item_type_id).set_id);
        EXPECT_EQ(flipped.item_type(item_type_id).set_size,
                  instance.item_type(item_type_id).set_size);
    }
    for (StackId s = 0; s < instance.number_of_stacks(); ++s) {
        EXPECT_EQ(flipped.set_id_of_stack(s), instance.set_id_of_stack(s));
        if (instance.set_id_of_stack(s) != -1) {
            EXPECT_EQ(flipped.set_size_of_stack(s),
                      instance.set_size_of_stack(s));
        }
    }
}

TEST(RectangleGuillotineSets, MutualExclusionStillRejectedAfterCopySupport)
{
    // The internal-copy exemption must NOT relax the user-input path:
    // an explicit STACK_ID + SET_ID on the same item still throws.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4, false, 3);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_bin_type(6000, 3210);

    EXPECT_THROW(instance_builder.build(), std::invalid_argument);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////// Branching scheme tests //////////////////////////////
////////////////////////////////////////////////////////////////////////////////

TEST(RectangleGuillotineSets, StackPredCrossSetBroken)
{
    // Three identical rows across two sets.
    // stack_pred_ must not link stacks across set boundaries.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_set(1, 2);
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    BranchingScheme branching_scheme(instance);
    // Should build without error. The cross-set links should be broken.
    auto root = branching_scheme.root();
    EXPECT_NE(root, nullptr);
}

TEST(RectangleGuillotineSets, StackPredDifferentSetSizeBroken)
{
    // Two rows with same geometry/copies but different SET_SIZE.
    // stack_pred_ must not link them.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 6);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_item_type(1000, 500, -1, 6);
    instance_builder.set_last_item_type_set(0, 3);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    BranchingScheme branching_scheme(instance);
    auto root = branching_scheme.root();
    EXPECT_NE(root, nullptr);
}

TEST(RectangleGuillotineSets, KnapsackTreeSearchOnlyAcceptsCompleteSubGroups)
{
    // Knapsack + sets: the bin fits 3 copies of the row (profit says
    // place 3), but better() must only retain sub-group-complete
    // nodes, so the best solution keeps 2 (one full pair).
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::Knapsack);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_bin_type(1000, 1600);
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters;
    optimize_parameters.optimization_mode
            = packingsolver::OptimizationMode::NotAnytimeSequential;
    auto output = optimize(instance, optimize_parameters);

    const Solution& best = output.solution_pool.best();
    EXPECT_EQ(best.number_of_items(), 2);
    EXPECT_EQ(best.item_copies(0) % 2, 0);
    EXPECT_TRUE(best.sets_feasible());
    EXPECT_TRUE(best.sets_complete());
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////// Solution sets checker tests ///////////////////////////
////////////////////////////////////////////////////////////////////////////////

namespace
{

// Bin 6000x3210, vertical first cut, 3 stages, NonExact — the house
// configuration. Items are placed via one d=1 strip per column with
// stacked d=2 children, so the node order in the certificate equals
// the placement order below.
InstanceBuilder sets_checker_instance_builder()
{
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    return instance_builder;
}

}

TEST(RectangleGuillotineSets, UpdateIndicatorsDetectsInterleaving)
{
    // Two rows of the same set interleaved A,B,A,B — the classic
    // twin-interleaving violation.
    InstanceBuilder instance_builder = sets_checker_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    SolutionBuilder solution_builder(instance);
    solution_builder.add_bin(0, 1, CutOrientation::Vertical);
    solution_builder.add_node(1, 1000);
    solution_builder.add_node(2, 500);
    solution_builder.set_last_node_item(0);
    solution_builder.add_node(2, 1000);
    solution_builder.set_last_node_item(1);
    solution_builder.add_node(2, 1500);
    solution_builder.set_last_node_item(0);
    solution_builder.add_node(2, 2000);
    solution_builder.set_last_node_item(1);
    Solution solution = solution_builder.build();

    EXPECT_FALSE(solution.sets_feasible());
    EXPECT_FALSE(solution.feasible());
    // Counts are pairwise complete — completeness is a separate axis.
    EXPECT_TRUE(solution.sets_complete());
}

TEST(RectangleGuillotineSets, UpdateIndicatorsAllowsStraddling)
{
    // A sub-group split across two bins is legal: plates are cut in
    // sequence, so the twins are still consecutive.
    InstanceBuilder instance_builder = sets_checker_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 2);
    instance_builder.set_last_item_type_set(0, 2);
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
    solution_builder.set_last_node_item(0);
    Solution solution = solution_builder.build();

    EXPECT_TRUE(solution.sets_feasible());
    EXPECT_TRUE(solution.feasible());
    EXPECT_TRUE(solution.sets_complete());
}

TEST(RectangleGuillotineSets, UpdateIndicatorsLenientTrailing)
{
    // A trailing incomplete sub-group does not flip feasible_ (bins
    // arrive incrementally); it only shows up in sets_complete().
    InstanceBuilder instance_builder = sets_checker_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    SolutionBuilder solution_builder(instance);
    solution_builder.add_bin(0, 1, CutOrientation::Vertical);
    solution_builder.add_node(1, 1000);
    solution_builder.add_node(2, 500);
    solution_builder.set_last_node_item(0);
    Solution solution = solution_builder.build();

    EXPECT_TRUE(solution.sets_feasible());
    EXPECT_TRUE(solution.feasible());
    EXPECT_FALSE(solution.sets_complete());
}

TEST(RectangleGuillotineSets, UpdateIndicatorsSparseSetIds)
{
    // SparseSetIds fixture shape (SET_IDs 50123/99999). The checker
    // state is sized number_of_sets() (dense) — it must index through
    // set_id_of_stack(), never through the sparse ItemType::set_id.
    InstanceBuilder instance_builder = sets_checker_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(50123, 2);
    instance_builder.add_item_type(600, 400, -1, 4);
    instance_builder.set_last_item_type_set(50123, 2);
    instance_builder.add_item_type(800, 300, -1, 6);
    instance_builder.set_last_item_type_set(99999, 3);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    // Complete run of set 99999 (3x item 2), then item 0 placed once
    // (mid-sub-group), then item 1 of the same set 50123 → violation.
    SolutionBuilder solution_builder(instance);
    solution_builder.add_bin(0, 1, CutOrientation::Vertical);
    solution_builder.add_node(1, 800);
    solution_builder.add_node(2, 300);
    solution_builder.set_last_node_item(2);
    solution_builder.add_node(2, 600);
    solution_builder.set_last_node_item(2);
    solution_builder.add_node(2, 900);
    solution_builder.set_last_node_item(2);
    solution_builder.add_node(1, 1800);
    solution_builder.add_node(2, 500);
    solution_builder.set_last_node_item(0);
    solution_builder.add_node(1, 2400);
    solution_builder.add_node(2, 400);
    solution_builder.set_last_node_item(1);
    Solution solution = solution_builder.build();

    EXPECT_FALSE(solution.sets_feasible());
}

TEST(RectangleGuillotineSets, UpdateIndicatorsHighCopyBin)
{
    // Replicated bins are k sequential plates.
    InstanceBuilder instance_builder = sets_checker_instance_builder();
    instance_builder.add_item_type(1000, 500, -1, 100);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_item_type(600, 400, -1, 100);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_bin_type(6000, 3210, -1, 200);
    Instance instance = instance_builder.build();

    // Clean: 50 copies of a self-contained pattern (one complete pair).
    {
        SolutionBuilder solution_builder(instance);
        solution_builder.add_bin(0, 50, CutOrientation::Vertical);
        solution_builder.add_node(1, 1000);
        solution_builder.add_node(2, 500);
        solution_builder.set_last_node_item(0);
        solution_builder.add_node(2, 1000);
        solution_builder.set_last_node_item(0);
        Solution solution = solution_builder.build();
        EXPECT_TRUE(solution.sets_feasible());
        EXPECT_TRUE(solution.sets_complete());
    }

    // Straddle across copies of the SAME replicated bin: one item per
    // plate, pair completes on the next plate — legal, needs replay.
    {
        SolutionBuilder solution_builder(instance);
        solution_builder.add_bin(0, 2, CutOrientation::Vertical);
        solution_builder.add_node(1, 1000);
        solution_builder.add_node(2, 500);
        solution_builder.set_last_node_item(0);
        Solution solution = solution_builder.build();
        EXPECT_TRUE(solution.sets_feasible());
        EXPECT_TRUE(solution.sets_complete());
    }

    // Violating pattern (A then B of the same set) — caught on the
    // first replayed plate.
    {
        SolutionBuilder solution_builder(instance);
        solution_builder.add_bin(0, 10, CutOrientation::Vertical);
        solution_builder.add_node(1, 1000);
        solution_builder.add_node(2, 500);
        solution_builder.set_last_node_item(0);
        solution_builder.add_node(1, 1600);
        solution_builder.add_node(2, 400);
        solution_builder.set_last_node_item(1);
        Solution solution = solution_builder.build();
        EXPECT_FALSE(solution.sets_feasible());
    }
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////// Algorithm selection tests ////////////////////////////
////////////////////////////////////////////////////////////////////////////////

TEST(RectangleGuillotineSets, TreeSearchForcedForSets)
{
    // Sets instance should run tree search and complete without error.
    InstanceBuilder instance_builder;
    instance_builder.read_item_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_basic" / "items.csv").string());
    instance_builder.read_bin_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_basic" / "bins.csv").string());
    instance_builder.read_parameters(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_basic" / "parameters.csv").string());
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters;
    optimize_parameters.optimization_mode
            = packingsolver::OptimizationMode::NotAnytimeSequential;
    auto output = optimize(instance, optimize_parameters);

    // All 8 items should be placed.
    EXPECT_EQ(output.solution_pool.best().number_of_items(), 8);
}

TEST(RectangleGuillotineSets, ExplicitSvcAllowedForBpplSets)
{
    // SVC + BPPL + sets is now allowed (per-algorithm gate); the run
    // must complete with a sets-feasible full placement.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters;
    optimize_parameters.optimization_mode
            = packingsolver::OptimizationMode::NotAnytimeSequential;
    optimize_parameters.use_sequential_value_correction = true;
    auto output = optimize(instance, optimize_parameters);

    const Solution& best = output.solution_pool.best();
    EXPECT_EQ(best.number_of_items(), 4);
    EXPECT_TRUE(best.sets_feasible());
    EXPECT_TRUE(best.sets_complete());
}

TEST(RectangleGuillotineSets, ExplicitColumnGenerationRejected)
{
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::BinPackingWithLeftovers);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters;
    optimize_parameters.optimization_mode
            = packingsolver::OptimizationMode::NotAnytimeSequential;
    optimize_parameters.use_column_generation = true;

    EXPECT_THROW(optimize(instance, optimize_parameters),
                 std::invalid_argument);
}

TEST(RectangleGuillotineSets, ExplicitColumnGeneration2Rejected)
{
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::Knapsack);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters;
    optimize_parameters.optimization_mode
            = packingsolver::OptimizationMode::NotAnytimeSequential;
    optimize_parameters.use_column_generation_2 = true;

    EXPECT_THROW(optimize(instance, optimize_parameters),
                 std::invalid_argument);
}

TEST(RectangleGuillotineSets, KnapsackExplicitSvcStillRejected)
{
    // The Knapsack OUTER objective stays tree-search-only for sets.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::Knapsack);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_bin_type(6000, 3210, -1, 2);
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters;
    optimize_parameters.optimization_mode
            = packingsolver::OptimizationMode::NotAnytimeSequential;
    optimize_parameters.use_sequential_value_correction = true;

    EXPECT_THROW(optimize(instance, optimize_parameters),
                 std::invalid_argument);
}

TEST(RectangleGuillotineSets, SingleBinKnapsackExplicitSvcStillRejected)
{
    // Single-bin instances zero the LOCAL svc flag before the gate
    // runs; the gate must read parameters.use_* (the raw request) and
    // still throw for Knapsack + sets.
    InstanceBuilder instance_builder;
    instance_builder.set_objective(Objective::Knapsack);
    instance_builder.set_number_of_stages(3);
    instance_builder.set_cut_type(CutType::NonExact);
    instance_builder.add_item_type(1000, 500, -1, 4);
    instance_builder.set_last_item_type_set(0, 2);
    instance_builder.add_bin_type(6000, 3210);
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters;
    optimize_parameters.optimization_mode
            = packingsolver::OptimizationMode::NotAnytimeSequential;
    optimize_parameters.use_sequential_value_correction = true;

    EXPECT_THROW(optimize(instance, optimize_parameters),
                 std::invalid_argument);
}

TEST(RectangleGuillotineSets, SingleRowSetSolves)
{
    // Single row with 20 copies, SET_SIZE=2.
    // Items are 1000x500 each (10M total), bin is 6000x3210 (19.26M).
    InstanceBuilder instance_builder;
    instance_builder.read_item_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_single_row" / "items.csv").string());
    instance_builder.read_bin_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_single_row" / "bins.csv").string());
    instance_builder.read_parameters(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_single_row" / "parameters.csv").string());
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters;
    optimize_parameters.optimization_mode
            = packingsolver::OptimizationMode::NotAnytimeSequential;
    auto output = optimize(instance, optimize_parameters);

    EXPECT_EQ(output.solution_pool.best().number_of_items(), 20);
}

TEST(RectangleGuillotineSets, MixedSetNonSetSolves)
{
    InstanceBuilder instance_builder;
    instance_builder.read_item_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_mixed" / "items.csv").string());
    instance_builder.read_bin_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_mixed" / "bins.csv").string());
    instance_builder.read_parameters(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_mixed" / "parameters.csv").string());
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters;
    optimize_parameters.optimization_mode
            = packingsolver::OptimizationMode::NotAnytimeSequential;
    auto output = optimize(instance, optimize_parameters);

    // All 13 items (8 set + 5 non-set) should be placed.
    EXPECT_EQ(output.solution_pool.best().number_of_items(), 13);
}

TEST(RectangleGuillotineSets, MultipleSetssolves)
{
    InstanceBuilder instance_builder;
    instance_builder.read_item_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_multiple" / "items.csv").string());
    instance_builder.read_bin_types(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_multiple" / "bins.csv").string());
    instance_builder.read_parameters(
            (fs::path("data") / "rectangleguillotine" / "tests"
             / "sets_multiple" / "parameters.csv").string());
    Instance instance = instance_builder.build();

    OptimizeParameters optimize_parameters;
    optimize_parameters.optimization_mode
            = packingsolver::OptimizationMode::NotAnytimeSequential;
    auto output = optimize(instance, optimize_parameters);

    // All 18 items should be placed.
    EXPECT_EQ(output.solution_pool.best().number_of_items(), 18);
}
