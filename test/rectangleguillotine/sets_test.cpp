#include "packingsolver/rectangleguillotine/instance_builder.hpp"
#include "packingsolver/rectangleguillotine/optimize.hpp"
#include "rectangleguillotine/branching_scheme.hpp"

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

TEST(RectangleGuillotineSets, ExplicitIncompatibleAlgorithmRejected)
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
